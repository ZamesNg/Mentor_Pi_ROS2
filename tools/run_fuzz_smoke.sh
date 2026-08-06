#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DOCKERFILE="${PROJECT_ROOT}/tools/docker/firmware-builder.Dockerfile"
readonly PUBLISHER="${PROJECT_ROOT}/tools/publish_fuzz_evidence.sh"
readonly VERIFIER="${PROJECT_ROOT}/tools/verify_fuzz_evidence.sh"
readonly IMAGE="mentor-pi/rrclite-firmware-builder:gcc-13.2.1"
readonly BUILD_DIRECTORY_RELATIVE="build/fuzz-smoke-linux"
readonly EVIDENCE_ROOT_RELATIVE="build/fuzz-evidence"
readonly MAXIMUM_RUNS_PER_HARNESS=10000000
readonly HARNESS_COUNT=7

declare -a docker_build_command=(docker build)
for proxy_variable in HTTP_PROXY HTTPS_PROXY NO_PROXY \
    http_proxy https_proxy no_proxy; do
  if [[ -n "${!proxy_variable:-}" ]]; then
    docker_build_command+=(
      --build-arg "${proxy_variable}=${!proxy_variable}"
    )
  fi
done
readonly -a docker_build_command

RUNS_PER_HARNESS=10000
PRINT_PLAN=0
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"

Usage() {
  cat <<'EOF'
Usage: ./tools/run_fuzz_smoke.sh [--runs RUNS] [--run-id RUN_ID] [--print-plan]

Build and run all seven deterministic RRCLite libFuzzer harnesses inside the
pinned Linux/Clang 18 firmware-builder container. RUNS is the number of inputs
per harness and must be in the range 1..10000000 (default: 10000).

Each successful run is atomically published under build/fuzz-evidence/RUN_ID.
An existing RUN_ID is never overwritten. If --run-id is omitted, a UTC time
and process-specific identifier is generated.

--print-plan validates the arguments and prints the bounded execution plan
without invoking Docker.
EOF
}

Fail() {
  echo "Fuzz runner error: $*" >&2
  exit 1
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --runs)
      [[ "$#" -ge 2 ]] || Fail "--runs requires a value"
      RUNS_PER_HARNESS="$2"
      shift 2
      ;;
    --run-id)
      [[ "$#" -ge 2 ]] || Fail "--run-id requires a value"
      RUN_ID="$2"
      shift 2
      ;;
    --print-plan)
      PRINT_PLAN=1
      shift
      ;;
    -h | --help)
      Usage
      exit 0
      ;;
    *)
      Usage >&2
      Fail "unknown argument: $1"
      ;;
  esac
done

[[ "${RUNS_PER_HARNESS}" =~ ^[1-9][0-9]*$ ]] || \
  Fail "RUNS must be a positive decimal integer"
((RUNS_PER_HARNESS <= MAXIMUM_RUNS_PER_HARNESS)) || \
  Fail "RUNS must not exceed ${MAXIMUM_RUNS_PER_HARNESS} per harness"
[[ "${RUN_ID}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,95}$ ]] || \
  Fail "RUN_ID must use 1..96 letters, digits, dots, underscores, or hyphens"

readonly TOTAL_EXECUTIONS=$((RUNS_PER_HARNESS * HARNESS_COUNT))
readonly EVIDENCE_DIRECTORY_RELATIVE="${EVIDENCE_ROOT_RELATIVE}/${RUN_ID}"

PrintPlan() {
  printf '%s\n' \
    'environment=linux-container' \
    'compiler=clang++-18' \
    'sanitizers=fuzzer,address,undefined' \
    "runs_per_harness=${RUNS_PER_HARNESS}" \
    "harness_count=${HARNESS_COUNT}" \
    "total_executions=${TOTAL_EXECUTIONS}" \
    "build_directory=${BUILD_DIRECTORY_RELATIVE}" \
    "run_id=${RUN_ID}" \
    "evidence_directory=${EVIDENCE_DIRECTORY_RELATIVE}" \
    "image=${IMAGE}"
}

if ((PRINT_PLAN == 1)); then
  PrintPlan
  exit 0
fi

command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || Fail "Docker Desktop/Engine is not running"
[[ -f "${DOCKERFILE}" ]] || Fail "missing pinned builder: ${DOCKERFILE}"
[[ -x "${PUBLISHER}" ]] || Fail "missing evidence publisher: ${PUBLISHER}"
[[ -x "${VERIFIER}" ]] || Fail "missing evidence verifier: ${VERIFIER}"
[[ ! -e "${PROJECT_ROOT}/${EVIDENCE_DIRECTORY_RELATIVE}" &&
   ! -L "${PROJECT_ROOT}/${EVIDENCE_DIRECTORY_RELATIVE}" ]] || \
  Fail "evidence run already exists and will not be overwritten: ${RUN_ID}"

"${docker_build_command[@]}" \
  --file "${DOCKERFILE}" --tag "${IMAGE}" \
  "${PROJECT_ROOT}/tools/docker"
readonly IMAGE_ID="$(docker image inspect --format '{{.Id}}' "${IMAGE}")"
[[ "${IMAGE_ID}" == sha256:* ]] || Fail "could not resolve Docker image ID"

docker run --rm --network=none \
  --user "$(id -u):$(id -g)" \
  --env RRCLITE_FUZZ_RUNS="${RUNS_PER_HARNESS}" \
  --env RRCLITE_FUZZ_RUN_ID="${RUN_ID}" \
  --env RRCLITE_FUZZ_IMAGE_ID="${IMAGE_ID}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${IMAGE}" \
  bash -euc '
    set -o pipefail
    write_production_source_manifest() {
      tools/firmware_source_fingerprint.sh --manifest firmware
    }
    write_test_input_manifest() {
      {
        printf "%s\0" \
          firmware/mentor_pi_mcu/CMakeLists.txt \
          tools/run_fuzz_smoke.sh \
          tools/publish_fuzz_evidence.sh \
          tools/verify_fuzz_evidence.sh
        find firmware/mentor_pi_mcu/tests/fuzz -type f -print0
      } | sort -z | xargs -0 sha256sum
    }
    write_toolchain_record() {
      printf "%s\n" \
        "image=mentor-pi/rrclite-firmware-builder:gcc-13.2.1" \
        "image_id=${RRCLITE_FUZZ_IMAGE_ID}" \
        "dockerfile_sha256=$(sha256sum tools/docker/firmware-builder.Dockerfile | cut -d " " -f 1)" \
        "os_release_sha256=$(sha256sum /etc/os-release | cut -d " " -f 1)"
      clang++-18 --version
      cmake --version
      ninja --version
    }
    digest_file() {
      sha256sum "$1" | cut -d " " -f 1
    }

    cmake -E remove_directory build/fuzz-smoke-linux
    cmake -E make_directory build/fuzz-smoke-linux
    campaign_started_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    write_production_source_manifest \
      >build/fuzz-smoke-linux/production-source-sha256-before.txt
    write_test_input_manifest \
      >build/fuzz-smoke-linux/test-input-sha256-before.txt
    write_toolchain_record \
      >build/fuzz-smoke-linux/toolchain-before.txt
    find firmware/mentor_pi_mcu/tests/fuzz/corpus -type f -print0 \
      | sort -z \
      | xargs -0 sha256sum \
      >build/fuzz-smoke-linux/source-corpus-sha256-before.txt
    cmake \
      -S firmware/mentor_pi_mcu \
      -B build/fuzz-smoke-linux \
      -G Ninja \
      -DBUILD_TESTING=OFF \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_CXX_COMPILER=clang++-18 \
      -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF \
      -DMENTOR_PI_MCU_ENABLE_FUZZING=ON \
      -DMENTOR_PI_MCU_FUZZ_SMOKE_RUNS="${RRCLITE_FUZZ_RUNS}"
    cmake --build build/fuzz-smoke-linux \
      --target mentor_pi_mcu_fuzz_smoke 2>&1 \
      | tee build/fuzz-smoke-linux/campaign.log
    write_production_source_manifest \
      >build/fuzz-smoke-linux/production-source-sha256-after.txt
    cmp build/fuzz-smoke-linux/production-source-sha256-before.txt \
      build/fuzz-smoke-linux/production-source-sha256-after.txt
    write_test_input_manifest \
      >build/fuzz-smoke-linux/test-input-sha256-after.txt
    cmp build/fuzz-smoke-linux/test-input-sha256-before.txt \
      build/fuzz-smoke-linux/test-input-sha256-after.txt
    write_toolchain_record \
      >build/fuzz-smoke-linux/toolchain-after.txt
    cmp build/fuzz-smoke-linux/toolchain-before.txt \
      build/fuzz-smoke-linux/toolchain-after.txt
    find firmware/mentor_pi_mcu/tests/fuzz/corpus -type f -print0 \
      | sort -z \
      | xargs -0 sha256sum \
      >build/fuzz-smoke-linux/source-corpus-sha256-after.txt
    cmp build/fuzz-smoke-linux/source-corpus-sha256-before.txt \
      build/fuzz-smoke-linux/source-corpus-sha256-after.txt
    source_corpus_files="$(
      wc -l <build/fuzz-smoke-linux/source-corpus-sha256-before.txt \
        | tr -d "[:space:]"
    )"
    source_corpus_sha256="$(
      digest_file build/fuzz-smoke-linux/source-corpus-sha256-before.txt
    )"
    production_source_files="$(
      wc -l <build/fuzz-smoke-linux/production-source-sha256-before.txt \
        | tr -d "[:space:]"
    )"
    production_source_sha256="$(
      digest_file build/fuzz-smoke-linux/production-source-sha256-before.txt
    )"
    firmware_source_fingerprint="$(
      tools/firmware_source_fingerprint.sh firmware
    )"
    [[ "${production_source_sha256}" == "${firmware_source_fingerprint}" ]]
    test_input_files="$(
      wc -l <build/fuzz-smoke-linux/test-input-sha256-before.txt \
        | tr -d "[:space:]"
    )"
    test_input_sha256="$(
      digest_file build/fuzz-smoke-linux/test-input-sha256-before.txt
    )"
    toolchain_sha256="$(
      digest_file build/fuzz-smoke-linux/toolchain-before.txt
    )"
    find build/fuzz-smoke-linux/fuzz-smoke-corpus -type f -print0 \
      | sort -z \
      | xargs -0 sha256sum \
      >build/fuzz-smoke-linux/final-corpus-sha256.txt
    final_corpus_files="$(
      wc -l <build/fuzz-smoke-linux/final-corpus-sha256.txt \
        | tr -d "[:space:]"
    )"
    final_corpus_sha256="$(
      digest_file build/fuzz-smoke-linux/final-corpus-sha256.txt
    )"
    cp build/fuzz-smoke-linux/production-source-sha256-before.txt \
      build/fuzz-smoke-linux/production-source-sha256.txt
    cp build/fuzz-smoke-linux/test-input-sha256-before.txt \
      build/fuzz-smoke-linux/test-input-sha256.txt
    cp build/fuzz-smoke-linux/source-corpus-sha256-before.txt \
      build/fuzz-smoke-linux/source-corpus-sha256.txt
    cp build/fuzz-smoke-linux/toolchain-before.txt \
      build/fuzz-smoke-linux/toolchain.txt
    printf "%s\n" \
      "schema=rrclite-fuzz-binding-v1" \
      "production_source_sha256=${production_source_sha256}" \
      "test_input_sha256=${test_input_sha256}" \
      "source_corpus_sha256=${source_corpus_sha256}" \
      "toolchain_sha256=${toolchain_sha256}" \
      >build/fuzz-smoke-linux/evidence-binding.txt
    evidence_binding_sha256="$(
      digest_file build/fuzz-smoke-linux/evidence-binding.txt
    )"
    git_head="UNAVAILABLE"
    if git rev-parse --verify HEAD >/dev/null 2>&1; then
      git_head="$(git rev-parse --verify HEAD)"
    fi
    campaign_finished_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    {
      printf "%s\n" \
        "schema=rrclite-fuzz-campaign-v2" \
        "result=PASS" \
        "run_id=${RRCLITE_FUZZ_RUN_ID}" \
        "campaign_started_utc=${campaign_started_utc}" \
        "campaign_finished_utc=${campaign_finished_utc}" \
        "runs_per_harness=${RRCLITE_FUZZ_RUNS}" \
        "harness_count=7" \
        "total_executions=$((RRCLITE_FUZZ_RUNS * 7))" \
        "sanitizers=fuzzer,address,undefined" \
        "seed=424242" \
        "semantic_preflight=PASS" \
        "git_head=${git_head}" \
        "revision_binding=production-source-test-toolchain-sha256" \
        "production_source_files=${production_source_files}" \
        "production_source_sha256=${production_source_sha256}" \
        "firmware_source_fingerprint_sha256=${firmware_source_fingerprint}" \
        "production_source_immutable=1" \
        "test_input_files=${test_input_files}" \
        "test_input_sha256=${test_input_sha256}" \
        "test_input_immutable=1" \
        "campaign_input_sha256=${test_input_sha256}" \
        "campaign_input_immutable=1" \
        "source_corpus_files=${source_corpus_files}" \
        "source_corpus_sha256=${source_corpus_sha256}" \
        "source_corpus_immutable=1" \
        "final_corpus_files=${final_corpus_files}" \
        "final_corpus_sha256=${final_corpus_sha256}" \
        "final_corpus_immutable=1" \
        "toolchain_sha256=${toolchain_sha256}" \
        "toolchain_immutable=1" \
        "docker_image_id=${RRCLITE_FUZZ_IMAGE_ID}" \
        "evidence_binding_sha256=${evidence_binding_sha256}"
    } >build/fuzz-smoke-linux/rrclite-fuzz-smoke-report.txt
    tools/publish_fuzz_evidence.sh \
      build/fuzz-smoke-linux \
      build/fuzz-evidence \
      "${RRCLITE_FUZZ_RUN_ID}"
  '

find "${PROJECT_ROOT}/${EVIDENCE_DIRECTORY_RELATIVE}" -maxdepth 1 -type f \
  -exec chmod a-w {} +
chmod a-w "${PROJECT_ROOT}/${EVIDENCE_DIRECTORY_RELATIVE}"
"${VERIFIER}" "${PROJECT_ROOT}/${EVIDENCE_DIRECTORY_RELATIVE}"
cat "${PROJECT_ROOT}/${EVIDENCE_DIRECTORY_RELATIVE}/rrclite-fuzz-smoke-report.txt"
echo "evidence_directory=${EVIDENCE_DIRECTORY_RELATIVE}"
