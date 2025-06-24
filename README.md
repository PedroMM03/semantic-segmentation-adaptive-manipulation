# semantic-segmentation-adaptive-manipulation
This project presents the development of a flexible robotic manipulation system based on computer vision, aimed at industrial applications. The main goal is to identify and locate objects through instance segmentation and automatically determine their pose (position and orientation) to enable their manipulation by a robotic arm.

## Hardware Requirements

- **NVIDIA Jetson AGX Xavier**
- **xArm6 - UFactory**
- **ZED Mini - Stereolabs**

## 🛠️ Development Environment Installation

### 📦 Prerequisites

- [Docker](https://www.docker.com/) installed in Jetson


### ⚙️ Setup Steps
# Install Docker buildx plugin (for ARM64 systems like Jetson)
mkdir -p ~/.docker/cli-plugins
curl -sSL https://github.com/docker/buildx/releases/download/v0.10.4/buildx-v0.10.4.linux-arm64 -o ~/.docker/cli-plugins/docker-buildx
chmod +x ~/.docker/cli-plugins/docker-buildx

# Confirm installation
docker buildx version

# Then, build the image using: 
sudo -E DOCKER_BUILDKIT=1 docker build \
  --build-arg PARALLEL_WORKERS=$(nproc) \
  -t jetson-dev-ros2 .

(This uses all CPU cores available for faster builds and enables caching through BuildKit)

## ZED SDK
https://www.stereolabs.com/en-tw/developers/release/4.2#82af3640d775
