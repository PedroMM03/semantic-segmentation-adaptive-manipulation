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

### Running the Docker Container

After building the Docker image, the next step is to **run a container**. The following instructions describe how to use and manage the `ros2_ws` (ROS 2 workspace) between the container and the host system.

---

#### 1️⃣ First run (internal `ros2_ws` inside the container)

Initially, you can run the container **without mounting any external ROS 2 workspace**:

```bash
sudo docker run -it --gpus all --ipc=host --network host --runtime=nvidia \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  --privileged jetson-yolo-zed-xarm-ros2:latest

```

#### 2️⃣ Copy the ROS 2 workspace to the host
While the container is running (from step 1), you can copy the ros2_ws directory to the host with:

```bash
sudo docker cp <container_id>:/root/ros2_ws /media/isa/data2/docker_scripts/
```

You can find the <container_id> using docker ps. 
Now the ROS 2 workspace exists on the host filesystem. You can edit it using Visual Studio Code or any other editor.

#### 3️⃣ Future runs (using shared workspace from the host)

In subsequent executions, it's best to mount the host copy of ros2_ws into the container using a volume. This ensures all changes persist and are accessible both inside and outside the container:

```bash
sudo docker run --rm -it --gpus all --ipc=host --network host --runtime=nvidia \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /media/isa/data2/docker_scripts/ros2_ws:/root/ros2_ws \
  --privileged jetson-yolo-zed-xarm-ros2:latest
```


> - The `-v /media/.../ros2_ws:/root/ros2_ws` option replaces the container’s default workspace with the **host version**.
> - Changes made inside the container are now saved **permanently in the host filesystem**.
> - This allows you to edit with VS Code on the Jetson, and build/run inside Docker.




> ⚠️ **Important Note**:  
> To allow the Docker container to access the X11 display server (needed for graphical applications like RViz or GUI tools), it is **recommended to run the following command before starting the container**:
>
> ```bash
> xhost +local:docker
> ```
>
> This avoids permission errors when the container tries to open graphical windows on the host.

ADD PROJECTS PACKAGES

Note: The package zed_vision requires a YOLO segmentation model to be manually added.

[Ultralytics YOLO models for segmentation](https://docs.ultralytics.com/es/tasks/segment/#models)


BUILD

HIERARCHY TREE WS

SEE COMAND USED DURING EXECUTION

HOW ADD CONTROL PANEL IN RVIZ2



SCRIPTS FOR FIGURES IN TEXT



# Other related links
## ZED SDK
https://www.stereolabs.com/en-tw/developers/release/4.2#82af3640d775
