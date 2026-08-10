ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential=12.10ubuntu1 \
       ca-certificates=20260601~24.04.1 \
       clang-18=1:18.1.3-1ubuntu1 \
       clang-format-18=1:18.1.3-1ubuntu1 \
       clang-tidy-18=1:18.1.3-1ubuntu1 \
       cmake=3.28.3-1build7 \
       git=1:2.43.0-1ubuntu7.3 \
       gzip=1.12-1ubuntu3.2 \
       ninja-build=1.11.1-2 \
       llvm-18=1:18.1.3-1ubuntu1 \
       python3=3.12.3-0ubuntu2.1 \
       ripgrep=14.1.0-1 \
       tar=1.35+dfsg-3ubuntu0.4 \
       unzip=6.0-28ubuntu4.1 \
       zsh=5.9-6ubuntu2 \
    && clang-format-18 --version | grep -F "version 18." \
    && cmake --version \
    && python3 --version \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
ENTRYPOINT []
CMD ["/bin/bash"]
