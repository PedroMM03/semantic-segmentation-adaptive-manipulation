# semantic-segmentation-adaptive-manipulation
This project presents the development of a flexible robotic manipulation system based on computer vision, aimed at industrial applications. The main goal is to identify and locate objects through instance segmentation and automatically determine their pose (position and orientation) to enable their manipulation by a robotic arm.

## Hardware Requirements

- **NVIDIA Jetson AGX Xavier**
- **xArm6 - UFactory**
- **ZED Mini - Stereolabs**

## 🛠️ Development Environment Installation

### 📦 Prerequisites

- **[Docker](https://www.docker.com/)** installed on the Jetson device  
  Used to isolate the build environment and manage dependencies inside a container.

- **[Visual Studio Code](https://code.visualstudio.com/)** (installed on the Jetson host)  
  Used for editing source code on the host system (ARM64).  
  The code is saved locally, while compilation and execution happen inside the Docker container.

💡 Note: The recommended workflow is to edit the code using Visual Studio Code on the Jetson host, 
with project files stored outside the container (in mounted volumes), and build/run everything 
inside the Docker container using mapped directories.



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
  -t jetson-yolo-zed-xarm-ros2

(This uses all CPU cores available for faster builds and enables caching through BuildKit)


sudo docker run -it --gpus all --ipc=host --network host --runtime=nvidia --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix -v /media/isa/data2/docker_scripts:/workspace -v /dev/bus/usb:/dev/bus/usb  --privileged jetson-yolo-zed-xarm-ros2:latest

Copy ROS 2 workspace into host folder

Ahora cada vez que llámenos a este contenedor debemos usar el ros2_ws del host no el del contendor para ello usamo volumen compartido:

sudo docker run -it --gpus all --ipc=host --network host --runtime=nvidia --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix -v /media/isa/data2/docker_scripts:/workspace -v /dev/bus/usb:/dev/bus/usb -v /media/isa/data2/docker_scripts/ros2_ws:/root/ros2_ws --privileged jetson-dev-ros2:latest



# Other related links
## ZED SDK
https://www.stereolabs.com/en-tw/developers/release/4.2#82af3640d775
