# ROS2 Humble + 相机雷达联合标定采集（精简版）
# 基础镜像自带：cv_bridge / image_transport / tf2_ros / pcl / rosbag2 / v4l2_camera
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# 系统工具 + ROS2 常用包（按需最小集）
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl vim nano tmux \
    python3-colcon-common-extensions python3-rosdep \
    python3-pip \
    v4l-utils libv4l-dev \
    libpcap-dev libyaml-cpp-dev libboost-all-dev \
    libopencv-dev python3-opencv \
    x11-apps \
    udev \
    && rm -rf /var/lib/apt/lists/*

# ROS2 关键包
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-v4l2-camera \
    ros-humble-usb-cam \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-message-filters \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# python 端 ros2 message 库（使用清华镜像，加速/防超时）
RUN pip3 install --no-cache-dir \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    --timeout 300 \
    
    
    opencv-python-headless

# rosdep（非必须，已注释）
# RUN rosdep init || true && rosdep update

ENV ROS_WS=/root/ros2_ws
RUN mkdir -p ${ROS_WS}
WORKDIR ${ROS_WS}

# 入口
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# 构建：在容器启动时手动执行
#   colcon build --symlink-install --packages-select camera_driver lidar_driver key_save
