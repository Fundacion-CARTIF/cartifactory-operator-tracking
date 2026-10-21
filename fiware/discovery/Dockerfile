FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    python3 \
    python3-pip \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U colcon-common-extensions vcstool

WORKDIR /ws

# Clone the discovery branch
RUN mkdir -p /ws/src && \
    git clone -b example/discovery-ws --recurse-submodules \
    https://github.com/eProsima/DDS-Enabler.git \
    /ws/src/ddsenabler

# Import dependencies, but skip ddsenabler because we already cloned the right branch
RUN vcs import /ws/src --skip-existing < /ws/src/ddsenabler/ddsenabler.repos

# Build DDS-Enabler with examples enabled
RUN colcon build --cmake-args -DCOMPILE_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release

# Run the discovery WebSocket example.
# It listens on ws://localhost:8080 by default.
CMD ["sh", "-lc", ". /ws/install/setup.sh && BIN=$(find /ws/install -type f -name ddsenabler_example_discovery | head -n 1) && test -n \"$BIN\" && exec \"$BIN\""]