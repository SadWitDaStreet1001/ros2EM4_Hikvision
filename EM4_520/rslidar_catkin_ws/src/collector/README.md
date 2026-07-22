# collector 功能包说明

`collector` 用于采集相机图像和 RoboSense 雷达点云。当前按硬件分类组织节点和 launch，方便分别调试 ZED 和 Hikvision。

## 目录结构

```text
collector/
  scripts/
    zed/
      zed_node.py
      zed_collector_node.py
    hikvision/
      hikvision_node.py
      hikvision_collector_node.py
  launch/
    zed/
      zed.launch
      zed_collect.launch
      zed_all.launch
    hikvision/
      hikvision.launch
      hikvision_photo.launch
      hikvision_collect.launch
      hikvision_all.launch
  data/
    zed/
    hikvision/
```

根目录 `launch/collector.launch` 是兼容入口，目前默认包含 `launch/zed/zed_all.launch`。

## 编译和环境

```bash
cd /workspace/code/EM4_520/rslidar_catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

每个新终端使用 ROS 节点前都需要：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
```

## 雷达启动

采集相机和点云前，先启动雷达驱动：

```bash
bash /workspace/code/EM4_520/start_rslidar_noetic_em4.sh
```

确认点云：

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /rslidar_points
rostopic echo -n 1 /rslidar_points/header
```

默认点云话题：

- `/rslidar_points`

## Hikvision 网络准备

当前 Hikvision 相机信息：

- 相机 IP：`192.168.1.64`
- RTSP 主码流：`rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101`
- RTSP 子码流：`rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/102`

当前 USB-RJ45 转接器：

- 固定接口名：`hikvision`
- MAC：`6c:1f:f7:df:9f:fb`
- 主机 IP：`192.168.1.103/32`

说明：雷达网卡使用 `192.168.1.102/24`，Hikvision 网卡使用 `192.168.1.103/32`，不要把两个网卡配置成同一个主机 IP。

首次配置时，建议在 Linux 宿主机执行一次永久固化脚本：

```bash
sudo /workspace/code/EM4_520/install_em4_network_persistent.sh
```

之后再次接入雷达和相机时，正常不需要手动改 IP。下面的脚本保留为运行时兜底，用于临时修正当前网卡状态。

配置 Hikvision 网卡：

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

正常会返回 `HTTP/1.1 200 OK`。

## Hikvision 采集

### 方式一：只查看相机并保存照片

这个节点不需要启动雷达，也不参与相机+雷达联合采集。它会直接读取 Hikvision RTSP，显示画面，并按键保存当前照片。
纯相机节点默认使用低延迟 RTSP 预览：后台持续读取相机流，只显示最新帧，避免窗口显示积压的旧画面。照片时间戳使用电脑收到当前 RTSP 帧时的本机时间。
默认预览窗口为原始图像的 `0.2` 倍、`10 fps` 刷新，目的是避免 noVNC/浏览器传输大画面时排队。按 `s` 保存时仍保存当前最新的原始分辨率图像。

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch
```

按键：

- `s`：保存当前画面
- `q`：退出

如果通过 Windows 的 VS Code Remote SSH 连接 Linux，Windows 端没有 X Server 时，建议使用仓库里的 noVNC 虚拟桌面脚本显示 OpenCV 窗口。虚拟桌面使用 `DISPLAY=:99`，noVNC 只监听 Linux 本机 `127.0.0.1:6080`，通过 VS Code 端口转发在 Windows 浏览器里访问。

首次在新机器上安装依赖：

```bash
sudo apt-get update
sudo apt-get install -y xvfb openbox x11vnc novnc websockify x11-xserver-utils x11-utils x11-apps
```

启动 noVNC 虚拟桌面：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/start_remote_desktop.sh
```

在 VS Code 的 `Ports` 面板转发端口 `6080`，然后在 Windows 浏览器打开：

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

刚打开时看到黑色空桌面是正常的，表示虚拟桌面已经连接，但还没有任何窗口。

运行纯相机采集。`hikvision_photo.launch` 默认把窗口显示到 noVNC 虚拟桌面 `:99`：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch
```

如果确实要显示到 Linux 物理桌面，可以手动覆盖：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch display:=:0
```

也可以使用封装脚本，它会先启动 noVNC 虚拟桌面，再运行纯相机采集：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/run_hikvision_photo_novnc.sh
```

如果已经手动启动了 noVNC 虚拟桌面，也可以自己指定 `DISPLAY=:99` 后运行原始命令：

```bash
export DISPLAY=:99
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch
```

停止 noVNC 虚拟桌面：

```bash
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/scripts/remote_desktop/stop_remote_desktop.sh
```

默认保存目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision/photos/YYYYMMDD/
```

照片命名：

```text
hikvision_<stamp>.png
photos.csv
```

### 方式二：同时启动相机和采集节点

推荐采集时使用。这个 launch 只启动 Hikvision 相机发布节点和采集节点，不启动雷达驱动；雷达需要你提前单独启动，并持续发布 `/rslidar_points`。

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch
```

默认显示行为：

- 只弹出 Hikvision 相机预览窗口。
- 不弹出雷达点云窗口；点云请用 RViz 查看。
- 联合采集默认使用 Hikvision 子码流 `Channels/102`，相机发布目标 `10 fps`，与当前雷达约 `10 Hz` 对齐。
- 按 `s` 保存前/中/后三帧图像，并为每帧匹配最近的 `/rslidar_points` 点云。
- 按 `d` 发起一次严格同步单帧保存：采集节点等待下一帧 `/rslidar_points`，以这帧雷达点云接收时间为同步目标，保存阈值内最近图像和该点云。
- 保存任务在后台线程执行，写大点云时不会再阻塞预览窗口；PCD 默认优先写二进制格式。

### 方式三：分开调试

只启动相机 RTSP 发布节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision.launch
```

只启动采集节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_collect.launch
```

### Hikvision 默认话题

- 图像话题：`/hikvision/image_raw`
- 点云话题：`/rslidar_points`

### 采集按键

采集节点默认只显示 Hikvision 图像窗口：

- `s`：保存前/中/后三帧图像，并为每帧匹配最近点云。
- `d`：雷达主导的严格单帧同步保存；采集节点等待下一帧雷达点云，再匹配最近图像。
- `q`：退出采集节点。

Hikvision 图像时间戳来自相机节点收到 RTSP 帧后的 `rospy.Time.now()`。雷达点云匹配使用采集节点收到 `PointCloud2` 时的电脑时间戳。按 `d` 时会先记录软件触发时间，然后等待下一帧雷达点云，以该点云接收时间作为同步目标去匹配最近图像。

默认最大同步阈值为 `20 ms`。如果图像和雷达点云时间差超过阈值，`s` 和 `d` 都会拒绝保存，不再用不满足阈值的数据兜底。

### Hikvision 保存结构

默认保存目录：

```text
/workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision
```

按 `s` 会生成三帧保存目录：

```text
data/hikvision/YYYYMMDD/group_<center_image_stamp>/
```

组内文件：

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

按 `d` 会生成单帧保存目录，目录名使用匹配到的图像时间戳：

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

PCD 会优先保存 `PointCloud2` 中所有可写成 PCD 的数值字段；字段连续且端序匹配时优先写 `DATA binary`，写盘速度明显快于 ASCII。不强制要求字段名必须是 `intensity`。无法写入 PCD 的字段会记录到 `metadata.json` 和 `metadata.csv` 的 `skipped_fields`。

单帧同步元数据中会记录 `trigger_stamp`、`sync_target_stamp`、`image_sync_target_delta_ms`、`cloud_sync_target_delta_ms` 和 `sync_delta_ms`。其中 `sync_target_stamp` 是作为同步目标的雷达点云接收时间，`sync_delta_ms` 是同一组图像时间戳和点云接收时间戳之间的时间差。

## Hikvision 常用参数

单独拍照时修改保存目录：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch \
  save_dir:=/workspace/data/hikvision_photos
```

纯相机预览默认使用 TCP 低缓冲 RTSP 参数，兼顾稳定性和低延迟。如果现场是稳定有线网络，仍然想测试 UDP，可以临时覆盖参数：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch \
  ffmpeg_capture_options:='rtsp_transport;udp|fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0|stimeout;5000000'
```

noVNC 中预览仍然延迟高时，优先继续降低预览刷新负载，例如 `6 fps` 和 `0.15` 缩放：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch \
  display_fps:=6 \
  preview_scale:=0.15
```

如果需要更大的本地物理显示窗口，可以手动调高：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch \
  display_fps:=15 \
  preview_scale:=0.35
```

联合采集默认已经使用子码流。如果需要手动指定子码流：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  rtsp_url:=rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/102
```

如果确实需要 4K 主码流保存，可以手动切回主码流，但延迟和 CPU 压力会明显上升：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  rtsp_url:=rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101 \
  preview_scale:=0.2 \
  display_fps:=10 \
  target_fps:=10
```

调整严格同步阈值为 `10 ms`：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  max_sync_age:=0.01
```

限制相机发布帧率为 `10 fps`：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  target_fps:=10
```

调整联合采集相机预览大小和刷新率：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  preview_scale:=0.2 \
  display_fps:=10
```

如果临时想恢复采集节点内置雷达俯视窗口，可以手动打开：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  show_lidar_preview:=true
```

推荐用 RViz 查看雷达点云：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/code/EM4_520/rslidar_catkin_ws/devel/setup.bash
rviz -d "/workspace/code/EM4_520/EM4-driver&SDK/rslidar_sdk/rviz/rviz.rviz"
```

RViz 中重点确认：

- `Fixed Frame`：`rslidar`
- 点云 `Topic`：`/rslidar_points`

修改保存目录：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch \
  data_dir:=/workspace/data/hikvision
```

## ZED 使用

只启动 ZED 相机发布节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/zed/zed.launch
```

只启动 ZED 采集节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/zed/zed_collect.launch
```

同时启动 ZED 相机和采集节点：

```bash
roslaunch /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/launch/zed/zed_all.launch
```

默认 ZED 图像话题：

- `/zed/image_raw`
- `/zed/left/image_raw`
- `/zed/right/image_raw`

默认 ZED 保存目录：

- `$(find collector)/data/zed`

## 检查命令

查看话题：

```bash
rostopic list
```

查看 Hikvision 图像频率：

```bash
rostopic hz /hikvision/image_raw
```

查看雷达点云频率：

```bash
rostopic hz /rslidar_points
```

查看最新保存结果：

```bash
find /workspace/code/EM4_520/rslidar_catkin_ws/src/collector/data/hikvision -maxdepth 3 -type f | sort | tail -40
```
