FROM eprosima/vulcanexus:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# System / ROS development dependencies
# ---------------------------------------------------------------------------

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    build-essential \
    git \
    cmake \
    pkg-config \
    libusb-1.0-0-dev \
    libudev-dev \
    libssl-dev \
    libgl1 \
    libglib2.0-0 \
    usbutils \
    ros-jazzy-ur \
    && rm -rf /var/lib/apt/lists/*

# RSUSB backend, matching the librealsense 2.58.4 used in the successful test.
# Disable the SDK DDS extension: ROS communication uses Vulcanexus Fast DDS.
ARG LIBREALSENSE_VERSION=2.58.4
RUN git clone --depth 1 --branch "v${LIBREALSENSE_VERSION}" \
        https://github.com/realsenseai/librealsense.git /tmp/librealsense-rsusb && \
    cmake -S /tmp/librealsense-rsusb -B /tmp/librealsense-rsusb/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/realsense-rsusb \
        -DCMAKE_INSTALL_LIBDIR=lib \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_WITH_DDS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_WITH_TM2=OFF && \
    cmake --build /tmp/librealsense-rsusb/build --parallel 2 && \
    cmake --install /tmp/librealsense-rsusb/build && \
    rm -rf /tmp/librealsense-rsusb

# ---------------------------------------------------------------------------
# Python dependencies
# ---------------------------------------------------------------------------

COPY requirements.txt /tmp/requirements.txt

RUN pip3 install \
    --break-system-packages \
    --ignore-installed \
    --no-cache-dir \
    -r /tmp/requirements.txt

# ---------------------------------------------------------------------------
# ROS workspace
# ---------------------------------------------------------------------------

WORKDIR /cartifactory_ws

COPY cartifactory_ws/src /cartifactory_ws/src

# Build the ROS wrapper AND its generated message types against Vulcanexus.
# Do not install the upstream ros-jazzy-realsense2-camera binary.
ARG REALSENSE_ROS_VERSION=4.58.4
RUN git clone --depth 1 --branch "${REALSENSE_ROS_VERSION}" \
        https://github.com/realsenseai/realsense-ros.git \
        /cartifactory_ws/src/realsense-ros

# Install dependencies declared by package.xml
RUN apt-get update && \
    apt-get install -y \
        ros-jazzy-ament-package \
        python3-colcon-common-extensions \
        ros-jazzy-ament-cmake && \
    rosdep update && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro jazzy \
        --skip-keys=librealsense2 \
        -y && \
    rm -rf /var/lib/apt/lists/*
# Build the pipeline and RealSense in the same Vulcanexus environment.
RUN source /opt/vulcanexus/jazzy/setup.bash && \
    export LD_LIBRARY_PATH="/opt/realsense-rsusb/lib:${LD_LIBRARY_PATH:-}" && \
    colcon build --symlink-install --parallel-workers 2 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -Drealsense2_DIR=/opt/realsense-rsusb/lib/cmake/realsense2

# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
    
# Importar grabaciones al volumen persistente.
RUN apt-get update && \
    apt-get install -y --no-install-recommends rsync && \
    rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
# Select RSUSB AFTER sourcing ROS/workspace, which modify LD_LIBRARY_PATH.
RUN sed -i 's/\r$//' /entrypoint.sh && \
    sed -i '/^exec /i export LD_LIBRARY_PATH="/opt/realsense-rsusb/lib:${LD_LIBRARY_PATH:-}"' /entrypoint.sh && \
    chmod +x /entrypoint.sh && \
    printf '%s\n' \
        'source /opt/vulcanexus/jazzy/setup.bash' \
        'if [ -f /cartifactory_ws/install/setup.bash ]; then source /cartifactory_ws/install/setup.bash; fi' \
        'export LD_LIBRARY_PATH="/opt/realsense-rsusb/lib:${LD_LIBRARY_PATH:-}"' \
        >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
