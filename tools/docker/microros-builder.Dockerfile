FROM microros/micro_ros_static_library_builder:jazzy@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       gcc-arm-none-eabi \
       libnewlib-arm-none-eabi \
       libstdc++-arm-none-eabi-dev \
    && rm -rf /var/lib/apt/lists/*
