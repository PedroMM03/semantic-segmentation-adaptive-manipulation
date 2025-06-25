#  Semantic Segmentation for Adaptive Manipulation with Industrial Robots
This project presents the development of a flexible robotic manipulation system based on computer vision, aimed at industrial applications. The main goal is to identify and locate objects through instance segmentation and automatically determine their pose (position and orientation) to enable their manipulation by a robotic arm.

## 1. 🛠️ Installation and Execution of the Developed System

### 📦 Prerequisites

- **NVIDIA Jetson AGX Xavier**
- **xArm6 - UFactory**
- **ZED Mini - Stereolabs**

- **[Docker](https://www.docker.com/)** installed on the Jetson device  
  Used to isolate the build environment and manage dependencies inside a container.

- **[Visual Studio Code](https://code.visualstudio.com/)** (installed on the Jetson host)  
  Used for editing source code on the host system (ARM64).  
  The code is saved locally, while compilation and execution happen inside the Docker container.

💡 Note: The recommended workflow is to edit the code using Visual Studio Code on the Jetson host, 
with project files stored outside the container (in mounted volumes), and build/run everything 
inside the Docker container using mapped directories.



### ⚙️ Setup Steps

#### Required Files for Building

Note: The ZED SDK installer is not included in this repository due to file size limitations. You must manually download it from the official website and place it alongside the Dockerfile.
https://www.stereolabs.com/en-tw/developers/release/4.2#82af3640d775
The version used in this project is ZED SDK for JetPack 5.1.2 (L4T 35.4) 4.2 (Jetson Xavier, Orin AGX/NX/Nano, CUDA 11.4)

```bash
Docker/
├── Dockerfile
├── ZED_SDK_Tegra_L4T35.4_v4.2.5.zstd.run
```

#### Install Docker buildx plugin (for ARM64 systems)
```bash
mkdir -p ~/.docker/cli-plugins
curl -sSL https://github.com/docker/buildx/releases/download/v0.10.4/buildx-v0.10.4.linux-arm64 -o ~/.docker/cli-plugins/docker-buildx
chmod +x ~/.docker/cli-plugins/docker-buildx
```
#### Confirm installation
```bash
docker buildx version
```
#### Then, build the image using: 
```bash
sudo -E DOCKER_BUILDKIT=1 docker build \
  --build-arg PARALLEL_WORKERS=$(nproc) \
  -t jetson-yolo-zed-xarm-ros2
```
(This uses all CPU cores available for faster builds and enables caching through BuildKit)

### Running the Docker Container

After building the Docker image, the next step is to **run a container**. The following instructions describe how to use and manage the `ros2_ws` (ROS 2 workspace) between the container and the host system.


#### 1️⃣ First run

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

#### 3️⃣ Future runs (using shared workspace from the host

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
> To allow the Docker container to access the X11 display server (needed for graphical applications like RViz), it is **recommended to run the following command before starting the container**:
>
> ```bash
> xhost +local:docker
> ```
>
> This avoids permission errors when the container tries to open graphical windows on the host.

### Add Custom ROS 2 Packages to the Workspace

Once the Docker container is running with the shared workspace (`ros2_ws`) from the host, you need to add the custom packages developed for this project.

1. **Locate the folder `ROS2_Packages/`** in the root of this repository. It contains the three packages used in this project:

   - `zed_vision`
   - `xarm6_controller`
   - `segmentation_panel`

2. **Copy or move the packages into your ROS 2 workspace:**

> ⚠️ Note: The `zed_vision` package requires a YOLO segmentation model to function properly.  
> Due to file size limitations, models are **not included** in this repository.

Please refer to [Ultralytics YOLO models for segmentation](https://docs.ultralytics.com/es/tasks/segment/#models) to download them manually.

For detailed instructions, see:  
`ROS2_Packages/zed_vision/resources/models/README_resources.md`

3. Your workspace should now have the following structure

```plaintext
ros2_ws/
├── src/
│   ├── zed_vision/
│   ├── xarm6_controller/
│   ├── segmentation_panel/
│   ├── moveit2/
│   ├── nmea_msgs/
│   ├── vision_opencv/
│   └── xarm_ros2/
├── install/
├── build/
└── log/
```
4. Build only the custom packages

```bash
cd ~/ros2_ws
colcon build --packages-select zed_vision xarm6_controller segmentation_panel

```

5. Source the workspace:

```bash

source install/setup.bash

```
---

### Execution Instructions

To run the full robotic manipulation system, you will need **four separate terminal windows**, each running a dedicated component inside the Docker container.
 
> All containers **share the same ROS 2 environment** 

Use the following command to start the container:

```bash
sudo docker run --rm -it --gpus all --ipc=host --network host --runtime=nvidia \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /media/isa/data2/docker_scripts/ros2_ws:/root/ros2_ws \
  --privileged jetson-yolo-zed-xarm-ros2:latest
```

#### Terminal 1 — Manipulator Connection
```bah
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.111
```

#### Terminal 2 — RViz Visualization
```bah
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.111 add_vacuum_gripper:=true
```

#### Terminal 3 — ZED Vision System
```bah
ros2 launch zed_vision zed_vision_launch.py
```
#### Terminal 4 — xArm6 Controller
```bah
ros2 launch xarm6_controller xarm6_controller_launch.py
```

### How to Add the Custom Control Panel And Annotated Frame in RViz2

1. Open the "Panels" menu
Click on Panels in the top bar of RViz2 and select Add New Panel.

![SETUPDEMO_1](https://github.com/user-attachments/assets/ee56e2d7-91e5-4d83-9f3c-3a43f9b1d365)

2. Locate the SegmentationPanel
In the new window, search for SegmentationPanel under the segmentation_panel package.

![SETUPDEMO_2](https://github.com/user-attachments/assets/5f1fed8f-3644-44ca-a0ea-afdbf8977b93)

The zed_vision node publishes an annotated image topic showing segmentation results. To visualize it:

1. In RViz2, click on Add in the Displays section.

2. Select by Topic.

3. Set the topic to /zed_vision/annotated_frame Image

![SETUPDEMO_3](https://github.com/user-attachments/assets/42327a5a-0249-47fd-ada6-8a82cdace398)

Final result:

![SETUPDEMO_4](https://github.com/user-attachments/assets/d79dce4e-b819-44ed-bc7e-8d9392e9e5c2)


---

## 2. Scripts Used to Generate Figures in the Report

This section includes complementary scripts used to generate some of the illustrative figures shown in the written report 

   - `report_scripts/svdsample.py — A minimal example demonstrating Singular Value Decomposition (SVD) on an image for visualization purposes used in the report`
   - `report_scripts/get_rpy_from_normal.ggb — GeoGebra 3D visualization of a reference frame aligned with a given surface normal`

---

## 3. 🔗 Additional Resources

### Screen Recording – SimpleScreenRecorder
Used to record demonstration videos.
Official site: https://www.maartenbaert.be/simplescreenrecorder/

```bash
sudo apt update
sudo apt install simplescreenrecorder
```

