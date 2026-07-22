# EM4 雷达 + Hikvision 相机容器交接文档

生成日期：2026-06-12

本文档面向完全不了解该项目的新同事，重点说明当前容器中 EM4 雷达、Hikvision 相机和联合采集节点的功能、启动方式、常用命令和排障方法。

## 1. 先看结论

当前容器主要用于：

- 启动 RoboSense EM4 雷达驱动，发布点云话题 `/rslidar_points`
- 连接 Hikvision 网络相机，通过 RTSP 获取图像
- 启动 Hikvision + EM4 联合采集节点，按键保存图像和对应点云
- 用 RViz 或命令行检查雷达点云
- 用 noVNC 在没有物理显示器的情况下显示 OpenCV/RViz 窗口

日常采集推荐使用两个终端：

```bash
# 终端 1：启动 EM4 雷达
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

```bash
# 终端 2：启动 Hikvision 相机和联合采集
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh
```

重要经验：

- 联合采集优先使用 `start_hikvision_all_em4.sh` 这个脚本。
- 不建议新同事直接用 `roslaunch hikvision_all.launch` 启动联合采集，因为直接 `roslaunch` 有时会遇到 ROS master、环境变量、相机网卡未配置等问题。
- `start_hikvision_all_em4.sh` 会自动检查 `/rslidar_points`、复用雷达启动脚本写入的 ROS 环境、检查 Hikvision 网卡和相机 HTTP 连通性，比直接 `roslaunch` 稳定。

## 2. 当前固定路径

项目根目录：

```text
/workspace/code/EM4_520
```

ROS 工作区：

```text
/workspace/code/EM4_520/rslidar_catkin_ws
```

主要功能包：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector
/workspace/code/EM4_520/EM4-driver&SDK/rslidar_sdk
```

## 3. 硬件和网络信息

### 3.1 EM4 雷达

当前雷达网络配置：

| 项目 | 当前值 |
| --- | --- |
| 雷达网卡名 | `lidar0` |
| 雷达网卡 MAC | `20:7b:d5:1a:07:0a` |
| 主机侧 IP | `192.168.1.102/24` |
| 雷达数据流端口 MSOP | `6699` |
| 雷达 DIFOP/标定端口 | `7766` |
| 雷达类型 | `RSEM4` |
| 点云话题 | `/rslidar_points` |
| 点云消息类型 | `sensor_msgs/PointCloud2` |
| 点云 frame_id | `rslidar` |
| ROS master | 默认 `http://127.0.0.1:11313` |

当前雷达配置文件：

```text
/workspace/code/EM4_520/EM4-driver&SDK/rslidar_sdk/config/config_em4_520.yaml
```

关键配置：

```yaml
lidar_type: RSEM4
host_address: 192.168.1.102
msop_port: 6699
difop_port: 7766
wait_for_difop: true
ros_send_point_cloud_topic: /rslidar_points
ros_frame_id: rslidar
```

已验证的雷达流量方向：

```text
192.168.1.200 -> 192.168.1.102:6699
192.168.1.200 -> 192.168.1.102:7766
```

### 3.2 Hikvision 相机

当前相机网络配置：

| 项目 | 当前值 |
| --- | --- |
| 相机网卡名 | `hikvision` |
| USB-RJ45 转接器 MAC | `6c:1f:f7:df:9f:fb` |
| 主机侧 IP | `192.168.1.103/32` |
| 相机 IP | `192.168.1.64` |
| 相机 HTTP | `http://192.168.1.64/` |
| 用户名 | `admin` |
| 密码 | `hyzx_hyzx` |
| RTSP 端口 | `554` |
| 图像话题 | `/hikvision/image_raw` |
| 图像 frame_id | `hikvision_camera` |

Hikvision 主码流：

```text
rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101
```

Hikvision 子码流：

```text
rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/102
```

联合采集默认使用子码流 `Channels/102`，主要是为了降低延迟和 CPU 压力。只查看相机或保存单张照片时默认使用主码流 `Channels/101`。

## 4. 容器内模块说明

### 4.1 顶层启动脚本

路径都在：

```text
/workspace/code/EM4_520
```

| 文件 | 功能 | 日常是否常用 |
| --- | --- | --- |
| `start_rslidar_noetic_em4.sh` | 配置 `lidar0`，启动 EM4 雷达驱动，发布 `/rslidar_points` | 常用 |
| `start_hikvision_all_em4.sh` | 检查雷达话题、配置 Hikvision 网卡、启动相机和联合采集节点 | 常用，推荐 |
| `start_hikvision_network.sh` | 单独配置 Hikvision 网卡和到相机的路由 | 偶尔排障 |
| `start_rviz_em4.sh` | 启动 RViz 查看 `/rslidar_points` | 调试时常用 |
| `reset_em4_ros_runtime.sh` | 清理 ROS master、roslaunch、雷达和采集残留进程 | 出问题时常用 |
| `install_em4_network_persistent.sh` | 在 Linux 宿主机永久固化两张网卡名、IP 和路由 | 首次部署时使用 |
| `prepare_host_x11_for_em4_container.sh` | 在宿主机放通容器访问 X11 显示 | 需要图形界面时使用 |
| `recreate_em4_container_with_rviz.sh` | 重建带 RViz/X11/GPU 权限的容器 | 需要重建容器时使用 |

### 4.2 EM4 雷达驱动模块

源码目录：

```text
/workspace/code/EM4_520/EM4-driver&SDK/rslidar_sdk
```

作用：

- 通过 UDP 接收 EM4 雷达数据
- 解析 MSOP 和 DIFOP 数据
- 发布 ROS 点云话题 `/rslidar_points`
- 提供 RViz 配置文件

核心文件：

| 文件 | 功能 |
| --- | --- |
| `config/config_em4_520.yaml` | 当前项目实际使用的 EM4 配置 |
| `launch/start.launch` | 启动 `rslidar_sdk_node` |
| `node/rslidar_sdk_node.cpp` | 雷达 ROS 节点入口 |
| `rviz/rviz.rviz` | RViz 点云显示配置 |

日常不要直接改 SDK 源码。需要改雷达 IP、端口、话题时，优先改 `config/config_em4_520.yaml`。

### 4.3 collector 采集功能包

功能包目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector
```

作用：

- 读取相机图像
- 订阅雷达点云
- 按键保存图像和点云
- 记录同步时间差和元数据

Hikvision 相关节点：

| 节点脚本 | 功能 |
| --- | --- |
| `scripts/hikvision/hikvision_node.py` | 从 Hikvision RTSP 读取图像，发布到 `/hikvision/image_raw` |
| `scripts/hikvision/hikvision_collector_node.py` | 订阅 `/hikvision/image_raw` 和 `/rslidar_points`，按键保存图像和点云 |
| `scripts/hikvision/hikvision_photo_node.py` | 只看相机画面并按 `s` 保存照片，不需要雷达 |

Hikvision launch 文件：

| launch | 功能 |
| --- | --- |
| `launch/hikvision/hikvision.launch` | 只启动 Hikvision 图像发布节点 |
| `launch/hikvision/hikvision_collect.launch` | 只启动联合采集保存节点 |
| `launch/hikvision/hikvision_all.launch` | 同时启动 Hikvision 图像发布节点和联合采集节点 |
| `launch/hikvision/hikvision_photo.launch` | 只启动相机拍照节点 |

ZED 相关内容也还保留在 `collector` 包中，但当前这份交接文档重点是 EM4 + Hikvision。ZED 入口在：

```text
rslidar_catkin_ws/src/collector/launch/zed/
```

### 4.4 noVNC 远程桌面模块

脚本目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop
```

| 脚本 | 功能 |
| --- | --- |
| `start_remote_desktop.sh` | 启动 Xvfb、openbox、x11vnc、websockify，提供 noVNC 页面 |
| `stop_remote_desktop.sh` | 停止 noVNC 虚拟桌面 |
| `run_hikvision_photo_novnc.sh` | 配置 Hikvision 网络，启动 noVNC，再启动相机拍照节点 |

默认 noVNC 地址：

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

默认虚拟显示：

```text
DISPLAY=:99
```

### 4.5 参考资料和非日常模块

| 路径 | 说明 |
| --- | --- |
| `README_RSLIDAR_EM4T.md` | 之前整理的 EM4 雷达运行说明，可作为补充参考 |
| `rslidar_catkin_ws/src/collector/README.md` | `collector` 功能包说明，包含 Hikvision 和 ZED 的更多参数 |
| `EM4-T_方形窗口 产品手册.pdf` | EM4-T 产品手册 |
| `wireshark抓包使用教程(1).pdf` | 抓包排查参考 |
| `RSView_Win_EM4_0331/` | Windows 端雷达查看工具相关文件，日常容器采集一般不用 |
| `EM4-driver&SDK/rs_driver-5c83dfe7/` | RoboSense 底层 driver/SDK 参考源码，日常不直接启动 |

## 5. 首次部署或换机器时要做的事

这部分通常在 Linux 宿主机做，不是在 Docker 容器里做。

### 5.1 固化雷达和相机网卡

只需要在宿主机执行一次：

```bash
cd /workspace/code/EM4_520
sudo ./install_em4_network_persistent.sh
```

它会做这些事：

- 把雷达 USB 网卡固定命名为 `lidar0`
- 把 Hikvision USB-RJ45 网卡固定命名为 `hikvision`
- 给 `lidar0` 固定 `192.168.1.102/24`
- 给 `hikvision` 固定 `192.168.1.103/32`
- 固定访问 `192.168.1.64` 时走 `hikvision` 网卡

检查：

```bash
ip -br addr show lidar0
ip -br addr show hikvision
ip route get 192.168.1.64
```

正常应类似：

```text
lidar0     UP  192.168.1.102/24
hikvision  UP  192.168.1.103/32
192.168.1.64 dev hikvision src 192.168.1.103
```

### 5.2 宿主机放通图形界面

如果要在容器里启动 RViz 或 OpenCV 窗口，宿主机需要允许容器访问 X11：

```bash
cd /workspace/code/EM4_520
./prepare_host_x11_for_em4_container.sh
```

如果原容器没有带 X11/GPU 权限，可以在宿主机重建：

```bash
cd /workspace/code/EM4_520
DISPLAY=${DISPLAY:-:0} XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority} ./recreate_em4_container_with_rviz.sh EM4_test
```

进入容器：

```bash
docker exec -it EM4_test bash
```

如果只是采集数据，不一定需要物理图形界面。没有图形界面时可以用 noVNC。

## 6. 日常启动流程

### 6.1 启动前确认

确认硬件：

- EM4 雷达已上电
- 雷达网线接到对应 USB 网卡
- Hikvision 相机已上电，PoE/网线正常
- Hikvision 相机接到 MAC 为 `6c:1f:f7:df:9f:fb` 的 USB-RJ45 转接器

确认当前目录：

```bash
cd /workspace/code/EM4_520
```

如果之前启动失败或不确定 ROS 状态，先清理：

```bash
bash /workspace/code/EM4_520/reset_em4_ros_runtime.sh
```

### 6.2 终端 1：启动 EM4 雷达

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

这个脚本会自动：

- 加载 ROS Noetic 环境
- 加载 `rslidar_catkin_ws/devel/setup.bash`
- 使用固定 ROS master：`http://127.0.0.1:11313`
- 写入运行时环境文件：`/tmp/em4_ros_runtime.env`
- 检查或重命名雷达网卡为 `lidar0`
- 配置 `lidar0` 为 `192.168.1.102/24`
- 启动 `rslidar_sdk start.launch`

正常日志会出现类似：

```text
Receive Packets From : Online LiDAR
Msop Port: 6699
Difop Port: 7766
Send PointCloud To : ROS
PointCloud Topic: /rslidar_points
RoboSense-LiDAR-Driver is running.....
```

该终端不要关闭，关闭后雷达点云也会停止。

### 6.3 终端 2：启动 Hikvision + EM4 联合采集

推荐命令：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh
```

这个脚本会自动：

- 读取 `/tmp/em4_ros_runtime.env`
- 检查当前 ROS master 里是否存在 `/rslidar_points`
- 如果找不到点云，会提示先启动雷达
- 检查 Hikvision 网卡是否是 `hikvision`
- 配置相机主机侧 IP `192.168.1.103/32`
- 检查访问 `192.168.1.64` 是否走 `hikvision`
- 用 `curl` 检查相机 HTTP 是否可达
- 启动 `hikvision_all.launch`

默认行为：

- 相机图像话题：`/hikvision/image_raw`
- 雷达点云话题：`/rslidar_points`
- 联合采集默认使用 Hikvision 子码流 `Channels/102`
- 相机发布目标帧率：`10 fps`
- 图像预览在检测到可用 `DISPLAY` 时默认开启；无图形环境时脚本会默认关闭
- 雷达俯视预览默认关闭，点云建议用 RViz 看
- 最大同步阈值：`0.02 s`，也就是 `20 ms`

按键：

| 按键 | 功能 |
| --- | --- |
| `s` | 保存前一帧、当前帧、后一帧三张图像，并分别匹配最近点云 |
| `d` | 软件触发单帧同步保存，等待下一帧雷达点云，再匹配最近图像 |
| `q` | 退出采集节点 |

注意：

- `s` 会等待下一帧图像，所以刚按下时不是立即完成。
- `d` 会等待下一帧雷达点云，并以该点云接收时间为同步目标。
- 如果图像和点云时间差超过 `20 ms`，当前代码会拒绝保存，避免保存不同步数据。
- 保存任务在后台线程执行，写 PCD 时通常不会卡住预览。

### 6.4 可选：单独确认 Hikvision 网络

如果相机打不开，可以单独运行：

```bash
bash /workspace/code/EM4_520/start_hikvision_network.sh
```

确认相机 Web 可达：

```bash
curl --noproxy '*' --interface hikvision -I http://192.168.1.64/
```

正常会返回类似：

```text
HTTP/1.1 200 OK
```

## 7. 数据保存位置

### 7.1 联合采集数据

默认目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision
```

按 `s` 保存三帧数据：

```text
data/hikvision/YYYYMMDD/group_<center_image_stamp>/
```

目录内容：

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

按 `d` 保存单帧数据：

```text
data/hikvision/YYYYMMDD/single_<image_stamp>/
```

目录内容：

```text
current_image_<stamp>.png
current_cloud_<stamp>.pcd
metadata.json
metadata.csv
```

查看最新保存结果：

```bash
find /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision -maxdepth 3 -type f | sort | tail -40
```

元数据中重点看：

- `image_file`：图像文件
- `pcd_file`：点云文件
- `image_header_stamp`：图像时间戳
- `cloud_recv_stamp`：采集节点收到点云的时间
- `sync_delta_ms`：图像和点云匹配时间差，越小越好
- `point_count`：点云点数
- `pcd_data`：PCD 保存格式，通常是 `binary`

### 7.2 只拍 Hikvision 照片的数据

默认目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision/photos/YYYYMMDD/
```

文件：

```text
hikvision_<stamp>.png
photos.csv
```

## 8. 常用检查命令

新开终端检查 ROS 时，先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
source /tmp/em4_ros_runtime.env 2>/dev/null || export ROS_MASTER_URI=http://127.0.0.1:11313
```

查看话题：

```bash
rostopic list
```

至少应看到：

```text
/rslidar_points
/hikvision/image_raw
```

查看点云频率：

```bash
rostopic hz /rslidar_points
```

当前 EM4 雷达一般约 `10 Hz`。

查看图像频率：

```bash
rostopic hz /hikvision/image_raw
```

查看点云头：

```bash
rostopic echo -n 1 /rslidar_points/header
```

正常 `frame_id` 应为：

```text
rslidar
```

查看点云尺寸：

```bash
rostopic echo -n 1 /rslidar_points/height
rostopic echo -n 1 /rslidar_points/width
```

抓雷达主数据流：

```bash
tcpdump -i lidar0 -nn 'udp port 6699'
```

抓雷达 DIFOP 数据流：

```bash
tcpdump -i lidar0 -nn 'udp port 7766'
```

查看 Hikvision 网卡：

```bash
ip -br addr show hikvision
ip route get 192.168.1.64
```

## 9. 查看点云和图形界面

### 9.1 使用 RViz

启动 RViz：

```bash
/workspace/code/EM4_520/start_rviz_em4.sh
```

RViz 里重点确认：

- `Fixed Frame` 是 `rslidar`
- `PointCloud2` 的 `Topic` 是 `/rslidar_points`

如果没有物理显示器或 X11 不通，可以使用 noVNC：

```bash
EM4_RVIZ_USE_NOVNC=1 /workspace/code/EM4_520/start_rviz_em4.sh
```

然后浏览器打开：

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

### 9.2 只看 Hikvision 画面并保存照片

推荐在容器里用封装脚本：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/run_hikvision_photo_novnc.sh
```

浏览器打开：

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

按键：

| 按键 | 功能 |
| --- | --- |
| `s` | 保存当前画面 |
| `q` | 退出 |

如果已经有可用显示环境，也可以直接：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch display:=:0
```

## 10. 直接 roslaunch 的方式

不推荐新同事优先使用直接 `roslaunch`，但调试时可能会用到。

使用前先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
source /tmp/em4_ros_runtime.env 2>/dev/null || export ROS_MASTER_URI=http://127.0.0.1:11313
```

只启动 Hikvision 图像发布节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision.launch
```

只启动 Hikvision 联合采集保存节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_collect.launch
```

同时启动 Hikvision 图像发布节点和采集节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch
```

如果直接 `roslaunch` 出现问题，优先回到推荐方式：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh
```

## 11. 常用参数覆盖

启动联合采集时，可以在脚本后面追加 roslaunch 参数。

例如修改保存目录：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh \
  data_dir:=/workspace/data/hikvision
```

切换到 4K 主码流，注意会增加延迟和写盘压力：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh \
  rtsp_url:=rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101 \
  preview_scale:=0.2 \
  display_fps:=10 \
  target_fps:=10
```

手动指定子码流：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh \
  rtsp_url:=rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/102
```

调整同步阈值为 `50 ms`：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh \
  max_sync_age:=0.05
```

打开采集节点内置的雷达俯视预览：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh \
  show_lidar_preview:=true
```

关闭图像预览：

```bash
EM4_SHOW_IMAGE_PREVIEW=false bash /workspace/code/EM4_520/start_hikvision_all_em4.sh
```

注意：关闭图像预览后没有 OpenCV 窗口，也就不能通过键盘按 `s`、`d`、`q` 触发保存或退出。正常人工采集时建议保持图像预览开启；无图形环境时先使用 noVNC。

## 12. 编译和改代码后更新

如果修改了 `collector` 包里的 Python 脚本、launch 或 CMake 配置，建议重新编译：

```bash
cd /workspace/code/EM4_520/rslidar_catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

每个新终端都需要：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
```

如果是日常采集，优先使用顶层脚本，脚本里已经处理了大部分环境加载。

## 13. 停止和重启

停止单个正在运行的节点：

```text
Ctrl+C
```

如果状态混乱，统一清理：

```bash
bash /workspace/code/EM4_520/reset_em4_ros_runtime.sh
```

如果还想清掉当前终端里的 ROS 环境变量，用 `source`：

```bash
source /workspace/code/EM4_520/reset_em4_ros_runtime.sh
```

清理后重新启动：

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

另开终端：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh
```

## 14. 常见问题排查

### 14.1 联合采集脚本提示找不到 `/rslidar_points`

原因：

- 雷达驱动还没启动
- 雷达启动失败
- 当前 ROS master 不是雷达脚本使用的 `http://127.0.0.1:11313`

处理：

```bash
bash /workspace/code/EM4_520/reset_em4_ros_runtime.sh
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

确认点云：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/em4_ros_runtime.env
rostopic hz /rslidar_points
```

### 14.2 Hikvision 网卡提示 `carrier=0`

原因通常是物理链路问题：

- 相机没上电
- PoE 供电异常
- 网线松动或损坏
- USB-RJ45 转接器没有链路灯

处理：

```bash
ip -br link show hikvision
cat /sys/class/net/hikvision/carrier
```

正常 `carrier` 应为 `1`。如果是 `0`，先处理硬件连接。

### 14.3 启动相机失败或 RTSP 打不开

先确认网络：

```bash
bash /workspace/code/EM4_520/start_hikvision_network.sh
curl --noproxy '*' --interface hikvision -I http://192.168.1.64/
```

如果 HTTP 不通，优先检查相机 IP、网卡路由和物理连接。

如果 HTTP 通但 RTSP 不通，检查账号密码和 RTSP 地址：

```text
rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101
rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/102
```

### 14.4 直接 `roslaunch hikvision_all.launch` 偶发启动异常

这是已知经验问题。直接 `roslaunch` 不会帮你完整处理这些事情：

- 是否已经启动雷达
- `/rslidar_points` 是否在同一个 ROS master
- 是否读取 `/tmp/em4_ros_runtime.env`
- Hikvision 网卡是否已经配置
- 相机路由是否正确

处理：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh
```

如果必须直接 `roslaunch`，先执行：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
source /tmp/em4_ros_runtime.env
bash /workspace/code/EM4_520/start_hikvision_network.sh
```

再执行 `roslaunch`。

### 14.5 RViz 或 OpenCV 窗口打不开

常见报错：

```text
cannot open display
qt.qpa.xcb: could not connect to display
No protocol specified
```

处理方案 1：用 noVNC：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/start_remote_desktop.sh
export DISPLAY=:99
```

浏览器打开：

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

处理方案 2：在宿主机放通 X11 后重建容器：

```bash
cd /workspace/code/EM4_520
./prepare_host_x11_for_em4_container.sh
DISPLAY=${DISPLAY:-:0} XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority} ./recreate_em4_container_with_rviz.sh EM4_test
```

### 14.6 有雷达 UDP 包，但没有点云

检查主机 IP：

```bash
ip -br addr show lidar0
```

应看到：

```text
lidar0 UP 192.168.1.102/24
```

检查 UDP：

```bash
tcpdump -i lidar0 -nn 'udp port 6699'
tcpdump -i lidar0 -nn 'udp port 7766'
```

如果能看到 `6699` 但没有 `7766`，雷达可能没有发 DIFOP/标定流。当前配置是 `wait_for_difop: true`，需要收到 DIFOP 后才正常发布点云。

### 14.7 按 `s` 或 `d` 后没有保存

常见原因：

- 图像和点云时间差超过 `max_sync_age`
- 默认阈值只有 `20 ms`
- 相机延迟过高，尤其是主码流 4K
- 点云话题频率异常
- 图像话题频率异常

检查：

```bash
rostopic hz /rslidar_points
rostopic hz /hikvision/image_raw
```

临时放宽阈值：

```bash
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh max_sync_age:=0.05
```

注意：阈值越大，同步精度越低。采集标定数据时不要随意放太大。

### 14.8 脚本提示 `Operation not permitted`

常见原因：

- 当前终端没有 root 权限
- 当前 Docker 容器没有 `--privileged`
- 当前容器没有使用 `--network host`
- 网卡实际在宿主机命名空间里，容器内不能直接改名或配置 IP

处理：

1. 先在容器里确认是否可以 sudo 或是否已经是 root。
2. 如果是网卡配置失败，回到 Linux 宿主机执行：

```bash
cd /workspace/code/EM4_520
sudo ./install_em4_network_persistent.sh
```

3. 如果容器没有设备和网络权限，按需要用宿主机脚本重建容器：

```bash
cd /workspace/code/EM4_520
DISPLAY=${DISPLAY:-:0} XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority} ./recreate_em4_container_with_rviz.sh EM4_test
```

## 15. 新同事接手建议

建议按下面顺序熟悉：

1. 先跑通 `start_rslidar_noetic_em4.sh`，确认 `/rslidar_points` 有数据。
2. 再跑通 `start_hikvision_all_em4.sh`，确认有 Hikvision 图像窗口。
3. 按 `s` 和 `d` 各保存一次，检查 `data/hikvision/YYYYMMDD/` 下面是否生成图像、PCD 和 metadata。
4. 用 `start_rviz_em4.sh` 看一次点云，确认 `Fixed Frame=rslidar`、`Topic=/rslidar_points`。
5. 只有在排障或二次开发时再看 `rslidar_sdk` 和 `collector/scripts` 里的源码。

优先记住三个命令：

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
bash /workspace/code/EM4_520/start_hikvision_all_em4.sh
bash /workspace/code/EM4_520/reset_em4_ros_runtime.sh
```
