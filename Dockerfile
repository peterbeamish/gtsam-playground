# Use Ubuntu image as base
FROM --platform=linux/amd64 ubuntu:22.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Bazel and dependencies using official APT repository
RUN apt-get update --allow-unauthenticated \
    && apt-get install -y --allow-unauthenticated \
    # Basic build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg \
    unzip \
    zip \
    openjdk-11-jdk \
    # GTSAM dependencies
    libeigen3-dev \
    libboost-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-regex-dev \
    libboost-program-options-dev \
    libboost-timer-dev \
    libboost-chrono-dev \
    libboost-date-time-dev \
    # Install Bazel using official APT repository
    && curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel-archive-keyring.gpg \
    && mv bazel-archive-keyring.gpg /usr/share/keyrings \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list \
    && apt-get update --allow-unauthenticated \
    && apt-get install -y --allow-unauthenticated bazel \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /var/cache/apt/archives/*

# Install GTSAM from source
RUN mkdir -p /tmp/gtsam_build \
    && cd /tmp/gtsam_build \
    && git clone https://github.com/borglab/gtsam.git \
    && cd gtsam \
    && git checkout 4.2.0 \
    && mkdir build \
    && cd build \
    && cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_SHARED_LIBS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DGTSAM_BUILD_DOC=OFF \
        -DGTSAM_INSTALL_GEORGIA_TECH_LICENSE=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
    && make -j$(nproc) \
    && make install

# Set up working directory
WORKDIR /workspace

# Create a non-root user for development
#RUN useradd -m -s /bin/bash developer && \
#    chown -R developer:developer /workspace
#USER developer

# Set up environment
ENV PATH="/home/developer/bin:$PATH"
ENV BAZEL_SH=/bin/bash
ENV JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64

# Default command
CMD ["/bin/bash"]
