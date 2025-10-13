# docker build --tag orb_slam3:latest .

FROM ubuntu:24.04

# Set DEBIAN_FRONTEND to noninteractive to avoid prompts during installation
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    wget \
    unzip \
    git \
    libopencv-dev \
    libeigen3-dev \
    libboost-serialization-dev \
    libssl-dev

# Copy the repository code
COPY . /ORB_SLAM3

# Set the working directory
WORKDIR /ORB_SLAM3

# Remove the build directory if it exists
RUN rm -rf build

# Build and test using the build script
RUN chmod +x build.sh && ./build.sh