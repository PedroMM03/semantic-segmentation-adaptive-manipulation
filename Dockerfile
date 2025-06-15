




# Imagen base JetPack 5.1.2 (L4T 35.4.1)
FROM nvcr.io/nvidia/l4t-jetpack:r35.4.1
ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8

# -----------------------------
# 1. Configuración de repositorios NVIDIA
# -----------------------------
RUN mkdir -p /tmp && chmod 1777 /tmp \
 && echo "deb https://repo.download.nvidia.com/jetson/common r35.4 main" \
      > /etc/apt/sources.list.d/nvidia.list \
 && echo "deb https://repo.download.nvidia.com/jetson/t234 r35.4 main" \
      >> /etc/apt/sources.list.d/nvidia.list \
 && sed -i 's|http://archive.ubuntu.com|http://ports.ubuntu.com|g' /etc/apt/sources.list \
 && sed -i '\#deb https://repo.download.nvidia.com/jetson/common#d' /etc/apt/sources.list \
 && sed -i '\#deb https://repo.download.nvidia.com/jetson/t234#d' /etc/apt/sources.list \
 && apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC


# -----------------------------
# 2. Dependencias básicas del sistema
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
     python3-pkg-resources \
     python3-distutils \
     python3-setuptools \
    build-essential \
    cmake \
    curl \
    git \
    wget \
    gnupg2 \
    lsb-release \
    locales \
    dirmngr \
    python3-pip \
    python3-rosdep \
    libopenmpi-dev \
    ocl-icd-opencl-dev \
    libopencv-dev \
    libusb-1.0-0-dev \
    zstd \
    iputils-ping \
    arp-scan \
    net-tools \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# 2.1 Instalar colcon-common-extensions y vcstool vía pip
RUN pip3 install --no-cache-dir \
    colcon-common-extensions \
    vcstool

# -----------------------------
# 3. Instalar ZED SDK v4.2.5
# -----------------------------
COPY ZED_SDK_Tegra_L4T35.4_v4.2.5.zstd.run /tmp/
RUN chmod +x /tmp/ZED_SDK_*.run && \
    /tmp/ZED_SDK_*.run -- silent --no-dialog && \
    rm /tmp/ZED_SDK_*.run && \
    pip3 install pyzed

# -----------------------------
# 4. Instalar Ultralytics, PyTorch, TorchVision y ONNX Runtime
# -----------------------------
RUN pip3 install --no-cache-dir ultralytics==8.3.91 \
 && pip3 install --no-cache-dir \
      https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.2.0-cp38-cp38-linux_aarch64.whl \
      https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.17.2+c1d70fe-cp38-cp38-linux_aarch64.whl \
 && wget -qO onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl \
      https://nvidia.box.com/shared/static/zostg6agm00fb6t5uisw51qi6kpcuwzd.whl \
 && pip3 install --no-cache-dir onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl numpy==1.23.5 \
 && rm onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl


# -----------------------------
# 5. Preparar el sistema para ROS 2 Foxy
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pkg-resources \
    python3-distutils \
    python3-setuptools \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 6. Instalar colcon-common-extensions (sin vcstool)
# -----------------------------
RUN pip3 install --no-cache-dir colcon-common-extensions

# -----------------------------
# 7. Agregar repositorio ROS 2 Foxy
# -----------------------------
RUN curl -sSL http://repo.ros2.org/repos.key | apt-key add - \
 && echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2-latest.list

# -----------------------------
# 8. Reparar sistema y forzar instalación de ROS 2 Foxy
# -----------------------------
RUN apt-get update \
 && apt-get install -f -y \
 && dpkg --configure -a \
 && apt-get install -y --no-install-recommends \
    -o Dpkg::Options::="--force-overwrite" \
    ros-foxy-desktop \
    ros-foxy-rviz2 \
    ros-foxy-gazebo-ros \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-tf2-tools \
    ros-foxy-tf2-eigen \
    ros-foxy-xacro \
    ros-foxy-robot-state-publisher \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-camera-info-manager \
    ros-foxy-visualization-msgs \
    ros-foxy-trajectory-msgs \
    ros-foxy-sensor-msgs \
 && apt-get clean && rm -rf /var/lib/apt/lists/*


# -----------------------------
# 7. Inicializar rosdep UNA sola vez
# -----------------------------
RUN rosdep update

# -----------------------------
# 8. Dependencias extra de ROS 2 que MoveIt2 / xArm / Gazebo necesitan
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-eigen3-cmake-module \
    ros-foxy-ament-cmake \
    ros-foxy-trajectory-msgs \
    ros-foxy-pluginlib \
    ros-foxy-geometry-msgs \
    ros-foxy-control-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-camera-info-manager \
    ros-foxy-controller-manager-msgs \
    ros-foxy-interactive-markers \
    ros-foxy-std-srvs \
    ros-foxy-backward-ros \
    ros-foxy-ament-pep257 \
    ros-foxy-moveit-resources-fanuc-moveit-config \
    ros-foxy-moveit-resources-panda-moveit-config \
    ros-foxy-joint-state-publisher \
    ros-foxy-geometric-shapes \
    ros-foxy-moveit-msgs \
    ros-foxy-ros-testing \
    ros-foxy-orocos-kdl \
    ros-foxy-object-recognition-msgs \
    ros-foxy-warehouse-ros \
    ros-foxy-control-toolbox \
    ros-foxy-launch-param-builder \
    ros-foxy-warehouse-ros-mongo \
    ros-foxy-srdfdom \
    ros-foxy-moveit-resources-pr2-description \
    ros-foxy-orocos-kdl \
    ros-foxy-gazebo-ros2-control \
    ros-foxy-gripper-controllers \
    libompl-dev \
    liborocos-kdl-dev \
    libomp-dev \
    python3-lxml \
    python-lxml \
    python3-empy \
    python3-jinja2 \
&& apt-get clean && rm -rf /var/lib/apt/lists/*

# Forzar versión compatible de Empy (3.3.4)
RUN pip3 install --no-cache-dir empy==3.3.4 && \
    rm -rf /usr/lib/python3/dist-packages/em*  # Eliminar versión conflictiva de apt


# -----------------------------
# 9. Crear workspace y clonar repositorios
# -----------------------------
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/ros-planning/moveit2.git -b foxy \
 && git -C moveit2 submodule update --init --recursive \
 && git clone https://github.com/ros-drivers/nmea_msgs.git \
 && git clone -b foxy https://github.com/xArm-Developer/xarm_ros2.git \
 && git -C xarm_ros2 submodule update --init --recursive \
 && git clone -b foxy https://github.com/ros-perception/vision_opencv.git

# 10. Instalar dependencias del workspace con rosdep
WORKDIR /root/ros2_ws
RUN bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src --ignore-src -y --rosdistro foxy --skip-keys 'orocos_kdl ompl pybind11_vendor urdfdom urdfdom_headers'"


# Define el número de workers (usa todos los núcleos)
ARG PARALLEL_WORKERS=8
# 11. Compilar workspace
RUN bash -c "source /opt/ros/foxy/setup.bash && \
    colcon build \
    --packages-ignore moveit_planners_ompl \
    --symlink-install \
    --parallel-workers 8 \
    --event-handlers console_cohesion+ \
    --continue-on-error"

# -----------------------------
# 12. Crear sourcing automático en bash
# -----------------------------
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc \
 && echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root
CMD ["bash"]
