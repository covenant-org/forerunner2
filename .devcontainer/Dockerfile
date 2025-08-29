# Use CUDA 12.8 Ubuntu 22.04 devel as base image
FROM nvidia/cuda:12.8.1-devel-ubuntu22.04

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Mexico_City

# Install essential packages for PX4 build and ZED SDK
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    cmake \
    build-essential \
    python3 \
    python3-pip \
    python3-dev \
    ninja-build \
    ccache \
    libxml2-dev \
    libxml2-utils \
    xsltproc \
    unzip \
    zip \
    libeigen3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libqt5gui5 \
    libqt5core5a \
    libqt5widgets5 \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libxml2-utils \
    pkg-config \
    libeigen3-dev \
    libopencv-dev \
    python3-empy \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-dev \
    python3-pip \
    python3-packaging \
    python3-wheel \
    python3-jinja2 \
    rsync \
    lsb-release \
    sudo \
    zstd \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for PX4
RUN pip3 install --no-cache-dir \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    empy \
    pyyaml \
    cerberus

# Create tools directory
WORKDIR /tools

# Clone and build PX4-Autopilot
RUN git clone https://github.com/covenant-org/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git submodule update --init --recursive && \
    bash ./Tools/setup/ubuntu.sh && \
    make px4_sitl_default

# Install ZED SDK
RUN wget -q  https://download.stereolabs.com/zedsdk/5.0/cu12/ubuntu22 -O ZED_SDK.zstd.run && \
    chmod +x ZED_SDK.zstd.run && \
    ./ZED_SDK.zstd.run -- --silent --accept-license && \
    rm ZED_SDK.zstd.run

# Download ZED AI models (without optimization for faster build)
RUN python3 /usr/local/zed/get_python_api.py

# Clean up to reduce image size
RUN apt-get autoremove -y && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* && \
    find /tools -name "*.o" -delete && \
    find /tools -name "*.a" -delete && \
    ccache --clear

# Set working directory to PX4
WORKDIR /tools/PX4-Autopilot

# Set default command
CMD ["/bin/bash"]
