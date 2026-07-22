# EM4 雷达 + Hikvision 相机 联合采集 (ROS2 Humble via Docker)

> **目标**：Ubuntu 20.04 主机 + Docker 容器跑 ROS2 Humble，驱动 EM4 雷达 + Hikvision 网络相机，按键保存同一帧点云和图像。
>
> **你只需要三个包**：
> 1. [`rslidar_sdk`](ros2_ws/src/rslidar_sdk/) — 驱动 EM4 雷达，发布 `/rslidar_points`
> 2. [`camera_driver`](ros2_ws/src/camera_driver/) — 驱动 Hikvision 相机，发布 `/hikvision/image_raw`
> 3. [`key_save`](ros2_ws/src/key_save/) — 按键保存当前最新一帧的图像和点云到磁盘
>
> 辅助包：[`rslidar_msg`](ros2_ws/src/rslidar_msg/)（rslidar_sdk 必需的消息包）。

## 1. 容器 & 进入方式

| 项目 | 值 |
|---|---|
| 容器名 | `ros2_humble_calib`（见 [docker-compose.yml:34](docker-compose.yml)） |
| 网络模式 | `host`（雷达 UDP 收包最稳） |
| 工作目录 | `/root/ros2_ws` |
| 数据目录 | `/root/bags` ⇄ 主机 `./bags` |

**进入容器**：

```bash
cd /home/shen/ROS2_Humble

# 一次性
xhost +local:docker

# 首次构建
make build

# 方式 A：前台进入（最常用）
make shell
# 容器里会自动 source /opt/ros/humble/setup.bash + /root/ros2_ws/install/setup.bash

# 方式 B：后台启动 + 进入
make up
make exec
```

**验证挂载**（在容器里）：

```bash
ls /root/bags          # 应该能看到主机 ./bags 内容
ls /dev/video*         # 设备透传
ip a | grep 192.168.1  # 应该能看到 lidar0/hikvision 网卡
```

## 2. 编译工作空间

容器内（已经 source 过）：

```bash
cd /root/ros2_ws
colcon build --symlink-install \
    --packages-select rslidar_msg rslidar_sdk camera_driver key_save
```

第一次编译会比较久（5~10 分钟），后面增量编译几秒。

## 3. 网络准备（**必做，在宿主机**）

EM4 雷达需要主机侧 `lidar0` 网卡 = `192.168.1.102/24`，Hikvision 相机需要 `hikvision` 网卡 = `192.168.1.103/32`。
这一步**必须在宿主机 shell 里执行**，不要在容器里：

```bash
# 雷达网卡（假设你的 USB-RJ45 网卡 MAC 是 20:7b:d5:1a:07:0a）
sudo ip link set lidar0 up
sudo ip addr add 192.168.1.102/24 dev lidar0

# 相机网卡（假设 MAC 是 6c:1f:f7:df:9f:fb）
sudo ip link set hikvision up
sudo ip addr add 192.168.1.103/32 dev hikvision

# 路由：访问相机 192.168.1.64 走 hikvision 网卡
sudo ip route add 192.168.1.64/32 dev hikvision src 192.168.1.103

# 防火墙放行（如果开了）
sudo iptables -I INPUT 1 -p udp --dport 6699 -j ACCEPT
sudo iptables -I INPUT 1 -p udp --dport 7766 -j ACCEPT
```

> 永久化配置见 [EM4_520/README_RSLIDAR_EM4T.md](EM4_520/README_RSLIDAR_EM4T.md) 里的 `install_em4_network_persistent.sh`。

## 4. 一键启动（容器内）

```bash
# 容器内（已经 source 过）
ros2 launch key_save all_em4.launch.py
```

会启动：
1. `rslidar_sdk_node` 拉 EM4 雷达 → `/rslidar_points`（frame: `rslidar`）
2. `hikvision_node` 拉 RTSP 流 → `/hikvision/image_raw`（frame: `hikvision_camera`）
3. 静态 TF：`rslidar` → `hikvision_camera`（粗值，标定后会覆盖）
4. `key_save` 节点：监听键盘

终端里看到 `[SAVE 000000] img=... pc=...pts dt=...ms` 就说明在正常保存。

## 5. 按键说明

| 按键 | 作用 |
|---|---|
| `s` / `Space` / `Enter` | 保存当前最新一帧的图像和点云 |
| `c` | 打印已保存数量 |
| `q` | 退出 |
| `h` / `?` | 打印帮助 |

> ⚠️ 终端必须是 TTY（`docker compose run --rm ros2 bash` 默认是 TTY；`docker exec` 也需要 `-it`）。
> 如果没有 TTY（比如后台启动 + 非交互），节点会自动订阅 `/key_save/trigger`（`std_msgs/Bool`），
> 外面发 `ros2 topic pub --once /key_save/trigger std_msgs/Bool '{data: true}'` 也可以触发保存。

## 6. 保存格式

按一次键保存一组，自动放到当天 session 子目录里：

```
bags/save/20250625_143022/
├── 000000/
│   ├── image_000000.png       # 图像（bgr8）
│   ├── points_000000.pcd      # 点云（PCD ASCII，XYZI）
│   └── stamp_000000.txt       # image_ts / lidar_ts / dt_ms
├── 000001/
│   ├── image_000001.png
│   ├── points_000001.pcd
│   └── stamp_000001.txt
└── ...
```

`stamp_xxx.txt` 内容示例：
```
image_ts = 1750865422.123456789
lidar_ts = 1750865422.135000000
dt_ms    = 11.544
save_ts  = 1750865422.180000000
```

## 7. 检查数据

```bash
# 容器内
ros2 topic hz /hikvision/image_raw
ros2 topic hz /rslidar_points

# 看点云
ros2 run pcl_ros pcd_to_pointcloud /tmp/test.pcd 1.0

# 看点云图像（用 RViz）
ros2 run rviz2 rviz2
# 添 PointCloud2，topic=/rslidar_points；添 Image，topic=/hikvision/image_raw
```

## 8. 单独启动每个包（调试用）

```bash
# 只起雷达
ros2 launch rslidar_sdk em4.launch.py

# 只起相机
ros2 launch camera_driver hikvision.launch.py

# 只起按键保存
ros2 launch key_save key_save.launch.py
```

## 9. 常见问题

| 问题 | 解决 |
|---|---|
| 容器内 `ros2 topic list` 没东西 | 没 source：`source /opt/ros/humble/setup.bash` |
| `rslidar_sdk` 启动后没数据 | `ip a` 看 `lidar0` 有没有 192.168.1.102；ping 192.168.1.200；检查 DIFOP 端口 7766 |
| 相机连不上 | 容器内 `ffprobe rtsp://admin:hyzx_hyzx@192.168.1.64:554/...`；宿主机执行 `curl http://admin:hyzx_hyzx@192.168.1.64/ISAPI/Streaming/channels` |
| `key_save` 不响应按键 | 终端不是 TTY（看 `tty` 命令）；用 `ros2 topic pub` 兜底 |
| `colcon build` 找不到 rslidar_msg | 确认 [rslidar_msg/](ros2_ws/src/rslidar_msg/) 在 src 下 |
| 时间同步 dt 太大 | 让雷达接 GPS/PTP；减小 key_save 的 `max_dt_ms` 参数 |

## 10. 关键参数

| 位置 | 参数 | 当前值 |
|---|---|---|
| [rslidar_sdk/config/config_em4.yaml](ros2_ws/src/rslidar_sdk/config/config_em4.yaml) | `lidar_type` | `RSEM4` |
| | `host_address` | `192.168.1.102` |
| | `msop_port` / `difop_port` | `6699` / `7766` |
| | `ros_send_point_cloud_topic` | `/rslidar_points` |
| | `ros_frame_id` | `rslidar` |
| [camera_driver/launch/hikvision.launch.py](ros2_ws/src/camera_driver/launch/hikvision.launch.py) | `rtsp_url` | `rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101` |
| | `image_topic` | `/hikvision/image_raw` |
| | `frame_id` | `hikvision_camera` |
| [key_save/launch/key_save.launch.py](ros2_ws/src/key_save/launch/key_save.launch.py) | `save_dir` | `/root/bags/save` |
| | `max_dt_ms` | `100.0` |
