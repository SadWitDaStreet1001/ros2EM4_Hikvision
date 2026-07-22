# EM4-T 雷达使用说明

本文档面向当前目录 `/workspace/code/EM4_520` 下已经配置好的 RoboSense EM4-T 雷达环境。

当前环境特点：

- 系统：`Ubuntu 20.04`
- ROS：`ROS Noetic`
- 驱动：`rslidar_sdk`
- 已验证可用的雷达网卡：`lidar0`，MAC：`20:7b:d5:1a:07:0a`
- 已验证可用的雷达主机 IP：`192.168.1.102/24`
- 已验证可用的主数据流端口：`6699`
- 已验证可用的 DIFOP/标定信息端口：`7766`
- 点云话题：`/rslidar_points`
- 点云类型：`sensor_msgs/PointCloud2`
- 已配置 Hikvision 网络相机采集入口：`collector/launch/hikvision/hikvision_all.launch`

## 目录说明

- 驱动源码：`/workspace/code/EM4_520/EM4-driver&SDK/rslidar_sdk`
- catkin 工作区：`/workspace/code/EM4_520/rslidar_catkin_ws`
- 启动脚本：`/workspace/code/EM4_520/start_rslidar_noetic_em4.sh`
- Hikvision 网卡配置脚本：`/workspace/code/EM4_520/start_hikvision_network.sh`
- collector 功能包说明：`/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/README.md`
- 当前使用配置：`/workspace/code/EM4_520/EM4-driver&SDK/rslidar_sdk/config/config_em4_520.yaml`

## 当前配置结论

这台 EM4-T 在当前环境下可以正常收包并发布点云。当前雷达会同时向主机发送 `6699` 主数据流和 `7766` DIFOP/标定信息流，配置中保持 `wait_for_difop: true`。

当前生效的关键参数：

- `lidar_type: RSEM4`
- `host_address: 192.168.1.102`
- `msop_port: 6699`
- `difop_port: 7766`
- `wait_for_difop: true`

## 首次配置主机

下面两类操作都要在 Linux 宿主机执行，不要在 Docker 容器里执行。当前 VS Code Remote SSH 进入的 shell 可能是容器；如果 `systemd-detect-virt` 输出 `docker`，说明你还没有在宿主机 shell。下面两个脚本检测到 Docker 容器时会直接退出，避免把宿主机配置误写到容器里。

### 1. 固化雷达和 Hikvision 网卡

只需要在宿主机执行一次：

```bash
cd /workspace/code/EM4_520
sudo ./install_em4_network_persistent.sh
```

这个脚本会做这些事：

- 固定雷达 USB 网卡 MAC `20:7b:d5:1a:07:0a` 为接口名 `lidar0`
- 固定 Hikvision USB 网卡 MAC `6c:1f:f7:df:9f:fb` 为接口名 `hikvision`
- 给 `lidar0` 固定 `192.168.1.102/24`
- 给 `hikvision` 固定 `192.168.1.103/32`
- 固定访问相机 `192.168.1.64` 时走 `hikvision` 网卡
- 优先写入 NetworkManager 配置；如果宿主机没有 NetworkManager，则写入 systemd-networkd 配置

执行完成后，拔插一次两个 USB 网卡，或重启主机，再检查：

```bash
ip -br addr show lidar0
ip -br addr show hikvision
ip route get 192.168.1.64
```

正常应看到：

```text
lidar0     UP  192.168.1.102/24
hikvision  UP  192.168.1.103/32
192.168.1.64 dev hikvision src 192.168.1.103
```

如果还没有做永久固化，或者临时换了网口，也可以继续使用运行时脚本：

```bash
/workspace/code/EM4_520/start_hikvision_network.sh
```

雷达启动脚本也会在启动驱动前自动检查并设置 `lidar0`：

```bash
/workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

### 2. 宿主机放通容器图形界面

如果容器里启动 `rviz` 或 OpenCV 窗口时报：

```text
No protocol specified
qt.qpa.xcb: could not connect to display
cannot open display
```

先在 Linux 宿主机桌面终端执行：

```bash
cd /workspace/code/EM4_520
./prepare_host_x11_for_em4_container.sh
```

如果要重新创建带 RViz 图形权限的容器，继续在宿主机执行：

```bash
cd /workspace/code/EM4_520
DISPLAY=${DISPLAY:-:0} XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority} ./recreate_em4_container_with_rviz.sh EM4_test
```

`recreate_em4_container_with_rviz.sh` 必须在宿主机桌面会话里运行；如果检测到当前 shell 在 Docker 容器内，或 `DISPLAY` 为空，它会直接退出。

进入容器：

```bash
docker exec -it EM4_test bash
```

容器里检查图形环境：

```bash
echo $DISPLAY
ls -l /tmp/.X11-unix
rviz
```

注意：如果只是看雷达点云，最稳妥的方式是在容器里启动雷达驱动，在宿主机桌面里运行 RViz；不要把 RViz 和采集节点强行绑在同一个容器里。

### 3. 容器内 noVNC 虚拟桌面运行 RViz

如果暂时不能把容器接到宿主机 X11，也可以用容器内 noVNC/Xvfb 虚拟桌面运行 RViz。这个方式适合临时调试，性能通常不如宿主机桌面直接运行 RViz。

在容器里执行：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/start_remote_desktop.sh
```

浏览器打开：

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

然后在容器里启动 RViz：

```bash
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
rviz -d "/workspace/code/EM4_520/EM4-driver&SDK/rslidar_sdk/rviz/rviz.rviz"
```

停止虚拟桌面：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/stop_remote_desktop.sh
```

## 启动前准备

### 1. 确认雷达接线

- 雷达通过接口盒 RJ45 接到主机
- 雷达已上电
- 雷达接口盒链路灯正常

### 2. 确认网卡链路

执行：

```bash
ip -br addr
ip -br link
```

正常情况下应看到类似输出：

```bash
lidar0  UP  192.168.1.102/24
```

说明：

- `lidar0` 是当前用于接雷达的接口
- 启动脚本会自动把它设置成 `192.168.1.102/24`

### 3. 确认 Hikvision 相机网卡

Hikvision 网络相机通过 USB-RJ45 转接器接入。当前转接器已固定为：

- 接口名：`hikvision`
- MAC：`6c:1f:f7:df:9f:fb`
- 主机 IP：`192.168.1.103/32`
- 相机 IP：`192.168.1.64`

说明：雷达主机 IP 保持 `192.168.1.102/24`，Hikvision 主机 IP 使用 `192.168.1.103/32`，避免两张网卡同时占用同一个 `192.168.1.102`。

执行：

```bash
bash /workspace/code/EM4_520/start_hikvision_network.sh
```

正常应看到类似：

```text
hikvision  UP  192.168.1.103/32
192.168.1.64 dev hikvision src 192.168.1.103
```

确认相机 Web 可达：

```bash
curl --noproxy '*' --interface hikvision -I http://192.168.1.64/
```

正常会返回：

```text
HTTP/1.1 200 OK
```

## 如何启动雷达驱动

直接执行：

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

如果脚本提示：

```text
[start_rslidar_noetic_em4] ROS master on http://127.0.0.1:11311 accepts TCP but does not respond
[start_rslidar_noetic_em4] using fallback ROS_MASTER_URI=http://127.0.0.1:11312
```

bash /workspace/code/EM4_520/reset_em4_ros_runtime.sh
说明默认 `11311` 端口上有一个不响应的 ROS master。脚本会自动改用 `11312` 启动雷达；后续启动 Hikvision 采集、`rostopic` 或 RViz 的终端，也要先执行：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
```

这个脚本会自动完成：

- `source /opt/ros/noetic/setup.bash`
- `source rslidar_catkin_ws/devel/setup.bash`
- 把 `lidar0` 的 IPv4 地址设置为 `192.168.1.102/24`
- 启动 `roslaunch rslidar_sdk start.launch`

正常启动后会看到类似日志：

```text
Receive Packets From : Online LiDAR
Msop Port: 6699
Difop Port: 7766
Send PointCloud To : ROS
PointCloud Topic: /rslidar_points
RoboSense-LiDAR-Driver is running.....
```

## 如何启动 Hikvision + 雷达联合采集

推荐按下面三个终端启动。

### 终端 1：启动雷达

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

### 终端 2：检查或临时配置 Hikvision 网卡

```bash
bash /workspace/code/EM4_520/start_hikvision_network.sh
```

如果已经在宿主机执行过 `install_em4_network_persistent.sh`，正常情况下这一步可以跳过。保留这个脚本是为了临时修正网卡状态；它只配置网卡，不会占用终端。执行完成后可以继续用该终端检查相机：

```bash
curl --noproxy '*' --interface hikvision -I http://192.168.1.64/
```

### 终端 3：启动 Hikvision 相机和采集节点

`hikvision_all.launch` 只启动 Hikvision 相机节点和采集节点，不启动雷达驱动。采集节点会订阅终端 1 发布的 `/rslidar_points`，并在按 `s` 或 `d` 时保存匹配点云。

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch
```

启动后默认只有一个窗口：

- Hikvision 图像窗口，默认小窗口低延迟预览
- 联合采集默认使用 Hikvision 子码流 `Channels/102`，目标 `15 fps`，优先保证低延迟

按键：

- `s`：保存前/中/后三帧图像，并为每帧匹配最近点云
- `d`：软件触发单帧同步保存，匹配离触发时间最近的图像和点云
- `q`：退出采集节点

说明：`s` 和 `d` 的写盘任务在后台线程执行，保存大点云时预览窗口不会再因为写 PCD 被阻塞。默认 PCD 优先使用二进制格式写入，比 ASCII 写入快很多。

如果确实需要 4K 主码流图像，可以手动覆盖回 `Channels/101`，但 4K H.264 解码和 PNG 写盘压力很大，预览延迟会明显上升：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  rtsp_url:=rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101 \
  preview_scale:=0.2 \
  display_fps:=10 \
  target_fps:=10
```

雷达点云不在采集节点里显示，建议用 RViz 单独查看 `/rslidar_points`。

Hikvision 默认图像话题：

```text
/hikvision/image_raw
```

雷达默认点云话题：

```text
/rslidar_points
```

### 只启动 Hikvision 相机节点

用于单独调试 RTSP 读取和图像发布：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision.launch
```

检查图像频率：

```bash
rostopic hz /hikvision/image_raw
```

### 只查看 Hikvision 画面并保存照片

这个节点不需要启动雷达，也不参与相机+雷达联合采集。适合单独查看 Hikvision 画面、按键保存照片：

#### 容器里运行，使用 noVNC 显示

如果你是在 Docker 容器里运行这个节点，推荐直接用封装脚本。它会先启动 `DISPLAY=:99` 虚拟桌面，再启动 `hikvision_photo.launch`，避免 `cannot open display: :99`：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/run_hikvision_photo_novnc.sh
```

在 Linux 宿主机浏览器打开：

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

如果需要手动分步执行，先在容器里启动虚拟桌面：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/start_remote_desktop.sh
```

确认 `:99` 可用：

```bash
xdpyinfo -display :99 | head
```

再启动单独相机采集节点：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch
```

如果要停止 noVNC 虚拟桌面：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/stop_remote_desktop.sh
```

#### 宿主机物理桌面直接显示

如果你不是在容器里，而是在 Linux 宿主机桌面终端直接运行，可以把窗口显示到物理桌面：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch display:=:0
```

如果仍然看到：

```text
Gtk-WARNING **: cannot open display: :99
```

说明你直接运行了默认 `display:=:99`，但没有先启动 noVNC 虚拟桌面。按上面的“容器里运行，使用 noVNC 显示”步骤处理。

按键：

- `s`：保存当前画面
- `q`：退出

默认保存目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision/photos/YYYYMMDD/
```

### 只启动 Hikvision 采集节点

用于相机节点已经启动时单独调试保存逻辑：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_collect.launch
```

## Hikvision 保存数据说明

默认保存目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision
```

按 `s` 会按日期创建三帧保存目录：

```text
data/hikvision/YYYYMMDD/group_<center_image_stamp>/
```

三帧目录包含：

```text
prev_image_<stamp>.png
center_image_<stamp>.png
next_image_<stamp>.png
prev_cloud_<stamp>.pcd
center_cloud_<stamp>.pcd
next_cloud_<stamp>.pcd
metadata.json
metadata.csv
```

按 `d` 会创建单帧保存目录，目录名使用匹配到的图像时间戳：

```text
data/hikvision/YYYYMMDD/single_<image_stamp>/
```

单帧目录包含：

```text
current_image_<stamp>.png
current_cloud_<stamp>.pcd
metadata.json
metadata.csv
```

时间同步策略：

- Hikvision 相机节点发布图像时，用电脑收到 RTSP 帧的 `rospy.Time.now()` 写入图像 `header.stamp`
- Hikvision 采集节点收到雷达 `PointCloud2` 时，也记录电脑接收时间戳
- 按 `s` 后保存前/中/后三帧图像
- 按 `d` 后记录软件触发时间，等待默认 `max_sync_age` 窗口，让触发点前后的图像和点云进入缓存
- `d` 单帧保存会分别选择离触发时间最近的图像和点云
- 每张图像会记录与所保存点云之间的时间差
- 默认最大同步阈值为 `50 ms`
- 如果前/后帧没有阈值内的点云，三张图像都会使用最接近中间图像时间戳的点云，并在元数据中记录 `used_center_cloud_fallback=true`

`d` 单帧同步元数据会记录 `trigger_stamp`、`image_trigger_delta_ms`、`cloud_trigger_delta_ms` 和 `sync_delta_ms`。其中 `sync_delta_ms` 是同一组图像时间戳和点云接收时间戳之间的时间差。

PCD 保存策略：

- 优先保存 `PointCloud2` 中所有可写成 PCD 的数值字段
- 字段连续且端序匹配时优先写 `DATA binary`，写盘速度明显快于 ASCII
- 不强制要求字段名必须是 `intensity`
- 无法写入 PCD 的字段会记录到元数据 `skipped_fields`

查看最新保存结果：

```bash
find /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision -maxdepth 3 -type f | sort | tail -40
```

查看某组元数据：

```bash
sed -n '1,120p' /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision/YYYYMMDD/group_*/metadata.csv
```

## 如何确认节点已启动

另开一个终端，执行：

```bash
source /opt/ros/noetic/setup.bash
rostopic list
```

正常情况下至少应看到：

```bash
/rosout
/rosout_agg
/rslidar_points
```

## 如何确认点云数据正常发布

### 1. 查看点云频率

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /rslidar_points
```

当前实测大约为：

```text
10 Hz
```

### 2. 查看点云消息头

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /rslidar_points/header
```

正常会看到类似：

```yaml
seq: 201
stamp:
  secs: 991
  nsecs: 200049000
frame_id: "rslidar"
```

### 3. 查看点云尺寸

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /rslidar_points/height
rostopic echo -n 1 /rslidar_points/width
```

当前环境实测可见非零输出，说明点云确实在发布。

## 如何抓包确认雷达数据

### 1. 抓主数据流 MSOP

```bash
tcpdump -i lidar0 -nn 'udp port 6699'
```

### 2. 抓配置流 DIFOP

```bash
tcpdump -i lidar0 -nn 'udp port 7766'
```

### 3. 当前已验证的流量方向

```text
192.168.1.200 -> 192.168.1.102:6699
192.168.1.200 -> 192.168.1.102:7766
```

如果抓不到 UDP 数据：

1. 先抓 ARP：

```bash
tcpdump -i lidar0 -nn arp
```

2. 如果能看到雷达在找 `192.168.1.102`，说明主机 IP 没配对
3. 直接重新运行启动脚本即可自动修正网卡 IP

## 如何查看点云

### 方案一：只看 ROS 话题

适合无图形界面的终端或容器环境：

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /rslidar_points
rostopic echo -n 1 /rslidar_points/header
```

### 方案二：用 RViz 查看点云

当前 `start.launch` 默认不启动 `rviz`，因为当前环境可能没有显示器。

如果你在有桌面的 ROS 主机上，需要手动启动。推荐使用封装脚本，它会加载当前雷达配置，并处理 `XDG_RUNTIME_DIR` / 容器软件 OpenGL 这类常见环境问题：

```bash
/workspace/code/EM4_520/start_rviz_em4.sh
```

直接运行 `rviz` 也可以，但如果看到 `QStandardPaths: XDG_RUNTIME_DIR not set` 或卡在 OpenGL 初始化，优先改用上面的脚本。

如果 RViz 打开后没有点云，先在 RViz 左侧确认：

- `Fixed Frame` 是 `rslidar`
- `PointCloud2` 的 `Topic` 是 `/rslidar_points`

同时在终端确认点云确实在发布：

```bash
rostopic hz /rslidar_points
rostopic echo -n 1 /rslidar_points/header
```

如果你是在 Docker/容器里执行，上面的命令只有在容器已经接入宿主机图形会话时才可用。

常见报错：

- `No protocol specified`
- `qt.qpa.xcb: could not connect to display`

这通常不是 `rviz` 本身损坏，而是容器没有获得宿主机 X11 显示权限，或没有正确传入 `DISPLAY` / `XAUTHORITY` / `/tmp/.X11-unix`。

更稳妥的做法：

- 容器里只运行雷达驱动和 ROS 节点
- 在宿主机桌面环境里单独运行 `rviz`

如果确实要在容器里启动 `rviz`，先在宿主机运行：

```bash
cd /workspace/code/EM4_520
./prepare_host_x11_for_em4_container.sh
DISPLAY=${DISPLAY:-:0} XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority} ./recreate_em4_container_with_rviz.sh EM4_test
docker exec -it EM4_test bash
```

如果脚本提示当前在 Docker 容器内，先退出容器，回到 Linux 宿主机桌面终端再执行。

容器启动时本质上至少需要这些 X11 参数：

```bash
xhost +local:
docker run --net=host \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$XAUTHORITY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $XAUTHORITY:$XAUTHORITY \
  ...
```

如果缺少这些挂载或权限，即使容器里安装了 `rviz`，也会出现无法连接显示器的错误。

RViz 中应关注：

- Fixed Frame：`rslidar`
- Topic：`/rslidar_points`

## 如何停止

在运行驱动的终端按：

```bash
Ctrl+C
```

## 一键清理 ROS 运行残留

如果雷达节点启动报错、`roslaunch` 卡住，或者怀疑有旧的 ROS master/节点残留，先执行：

```bash
bash /workspace/code/EM4_520/reset_em4_ros_runtime.sh
```

这个脚本会清理当前可见环境里的 ROS/EM4 运行进程，包括：

- `roscore` / `rosmaster` / `roslaunch`
- `rosout` / `rostopic` / `rosnode` / `rosbag`
- `rslidar_sdk_node`
- `rviz`
- Hikvision 采集相关节点

如果还想顺便清掉当前终端里的 `ROS_MASTER_URI`、`ROS_HOSTNAME`、`ROS_IP`，用 `source` 执行：

```bash
source /workspace/code/EM4_520/reset_em4_ros_runtime.sh
```

注意：如果脚本提示 `11311` 或 `11312` 端口仍然被占用、但没有可见 PID，说明占端口的进程大概率在 Linux 宿主机或另一个命名空间里。此时要回到 Linux 宿主机终端执行：

```bash
pkill -f roscore
pkill -f rosmaster
pkill -f roslaunch
```

然后再回到容器/当前终端启动雷达：

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

## 常见问题

### 1. 启动卡在 `bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh`

现象：

- 终端只看到启动命令，或者只打印到 `lidar0 UP 192.168.1.102/24`
- 后面很久没有 `RoboSense-LiDAR-Driver is running.....`

常见原因：

- 默认 `ROS_MASTER_URI=http://127.0.0.1:11311`
- `11311` 端口上有旧的或宿主机里的 ROS master 进程，TCP 能连接，但 XML-RPC 不响应
- `roslaunch` 会卡在检查这个 master 的阶段

处理：

- 当前脚本已加入启动前检查
- 遇到这种情况会自动改用 `ROS_MASTER_URI=http://127.0.0.1:11312`
- 如果后续还要启动采集节点、`rostopic` 或 RViz，先在对应终端执行：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
```

也可以回到 Linux 宿主机终端，停止旧的 ROS master 后再使用默认 `11311`：

```bash
pkill -f roscore
pkill -f rosmaster
```

### 2. 启动脚本报 `ROS_MASTER_URI: unbound variable`

原因：

- 旧版本脚本在 `set -u` 下加载 ROS 环境会报错

处理：

- 当前脚本已修复
- 直接使用最新版 `start_rslidar_noetic_em4.sh`

### 3. 启动脚本报 `could not connect to display`

原因：

- 启动时尝试自动拉起 `rviz`
- 当前环境没有图形显示

处理：

- 当前 `start.launch` 已改为默认不启动 `rviz`
- 如果在宿主机桌面运行，手动运行 `rviz`
- 如果必须在容器里运行，先在宿主机执行 `/workspace/code/EM4_520/prepare_host_x11_for_em4_container.sh`，再用 `/workspace/code/EM4_520/recreate_em4_container_with_rviz.sh EM4_test` 重建容器

### 4. 有 UDP 包，但不出点云

当前这台 EM4-T 的处理结论：

- 网络正常
- `6699/7766` 都正常到达
- 配置保持 `wait_for_difop: true`，等待 `7766` 标定信息后发布点云

已经在当前配置中处理好，无需再次手改。

### 5. 再次接入雷达后，是否还要重新改网卡 IP

正常不需要手动改。

原因：

- 已经可以在宿主机执行 `sudo /workspace/code/EM4_520/install_em4_network_persistent.sh` 永久固化网卡命名、IP 和相机路由
- 启动脚本每次也会自动把 `lidar0` 设成 `192.168.1.102/24`，作为运行时兜底
- 如果相机连不通，可以手动执行 `/workspace/code/EM4_520/start_hikvision_network.sh` 立即修正 `hikvision` 网卡

## 常用命令速查

首次在宿主机固化两个网卡：

```bash
cd /workspace/code/EM4_520
sudo ./install_em4_network_persistent.sh
```

宿主机放通 Docker 图形界面：

```bash
cd /workspace/code/EM4_520
./prepare_host_x11_for_em4_container.sh
```

启动驱动：

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

配置 Hikvision 网卡：

```bash
bash /workspace/code/EM4_520/start_hikvision_network.sh
```

启动 Hikvision 相机和采集节点：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch
```

只启动 Hikvision 相机节点：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision.launch
```

只查看 Hikvision 画面并保存照片：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch
```

只启动 Hikvision 采集节点：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_collect.launch
```

查看话题：

```bash
source /opt/ros/noetic/setup.bash
rostopic list
```

查看 Hikvision 图像频率：

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /hikvision/image_raw
```

查看点云频率：

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /rslidar_points
```

查看点云头：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /rslidar_points/header
```

抓主数据流：

```bash
tcpdump -i lidar0 -nn 'udp port 6699'
```

抓配置流：

```bash
tcpdump -i lidar0 -nn 'udp port 7766'
```

确认 Hikvision 相机 Web 可达：

```bash
curl --noproxy '*' --interface hikvision -I http://192.168.1.64/
```

查看 Hikvision 最新保存结果：

```bash
find /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision -maxdepth 3 -type f | sort | tail -40
```

启动 RViz：

```bash
/workspace/code/EM4_520/start_rviz_em4.sh
```

如果这是容器内 shell，请优先在宿主机桌面终端运行这条命令，而不是在容器里直接运行。
