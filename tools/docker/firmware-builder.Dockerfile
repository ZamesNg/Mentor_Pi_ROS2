FROM microros/micro_ros_static_library_builder:jazzy@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       cmake \
       ninja-build \
       gcc-arm-none-eabi \
       libnewlib-arm-none-eabi \
       libstdc++-arm-none-eabi-dev \
    && test "$(arm-none-eabi-gcc -dumpfullversion)" = "13.2.1" \
    && rm -rf /var/lib/apt/lists/*

ENV SOURCE_DATE_EPOCH=0
WORKDIR /workspace
ENTRYPOINT []
CMD ["/bin/bash"]
