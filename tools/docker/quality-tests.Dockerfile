FROM ubuntu:24.04@sha256:561618e2c15bf2397621dd04f96926663a3b5616c189cf7e38db7e82f5c538ea

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential \
       ca-certificates \
       clang-format-18 \
       cmake \
       git \
       gzip \
       ninja-build \
       python3 \
       ripgrep \
       tar \
       unzip \
    && clang-format-18 --version | grep -F "version 18." \
    && cmake --version \
    && python3 --version \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
ENTRYPOINT []
CMD ["/bin/bash"]
