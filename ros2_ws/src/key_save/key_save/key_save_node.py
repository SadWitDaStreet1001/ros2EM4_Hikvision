"""
按键保存同一帧相机+雷达数据。

运行：
  ros2 run key_save key_save
  # 或
  ros2 launch key_save key_save.launch.py save_dir:=/root/bags/run1

操作：
  - 在终端里按 s / 空格 / 回车 → 保存当前缓存的最新一帧
  - 按 q → 退出
  - 按 c → 显示当前已保存数量

保存格式（每次按一下保存一组，配对命名）：
  <save_dir>/<seq>/
    image_<seq>.png          # 图像
    points_<seq>.pcd         # 点云（PCD ASCII 格式）
    stamp_<seq>.txt          # 时间戳（image_ts lidar_ts dt）

适用：
  - 相机/雷达联合标定的棋盘板数据采集
  - 任意需要"按需保存同步数据"的场景

注意：
  - "同一帧" 是软件时间同步策略：取「按按键时刻往前回溯的最近一帧」，
    输出的 dt（image_ts - lidar_ts）就是这对配对的时间差。
  - 如果对同步精度要求高（<20ms），需要让相机支持硬件触发，或减小话题延迟。
"""
import os
import sys
import threading
import time
import select
import termios
import tty
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2


def _now_ns():
    return time.time_ns()


def _save_pcd(path: str, points: np.ndarray):
    """points: (N, 3) 或 (N, 4) [x,y,z,i]"""
    n = points.shape[0]
    has_i = points.shape[1] >= 4
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z' + (' intensity' if has_i else '') + '\n')
        f.write('SIZE 4 4 4' + (' 4' if has_i else '') + '\n')
        f.write('TYPE F F F' + (' F' if has_i else '') + '\n')
        f.write('COUNT 1 1 1' + (' 1' if has_i else '') + '\n')
        f.write(f'WIDTH {n}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {n}\n')
        f.write('DATA ascii\n')
        if has_i:
            for p in points:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.4f}\n')
        else:
            for p in points:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


class KeySave(Node):
    def __init__(self):
        super().__init__('key_save')

        # ---- 参数 ----
        self.declare_parameter('camera_topic',     '/camera/image_raw')
        self.declare_parameter('lidar_topic',      '/lidar/points_raw')
        self.declare_parameter('save_dir',         '/root/bags/save')
        self.declare_parameter('image_encoding',   'bgr8')   # bgr8 / rgb8 / mono8
        self.declare_parameter('max_latest_age_s', 0.5)      # 最新帧超过这个时间就报警
        self.declare_parameter('auto_session',     True)     # 启动时自动创建 session 子目录
        self.declare_parameter('session_name',     '')       # 空 = 用时间戳
        self.declare_parameter('max_dt_ms',        100.0)    # image-lidar 配对时间差阈值

        # ---- 缓存 ----
        self._lock = threading.Lock()
        self._latest_image = None       # (cv_image, stamp_sec)
        self._latest_pc    = None       # (np.ndarray Nx3/4, stamp_sec)
        self._bridge = CvBridge()

        # ---- 订阅（depth=1，只要最新的）----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub_img = self.create_subscription(
            Image, self.get_parameter('camera_topic').value,
            self._on_image, qos)
        self.sub_pc = self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value,
            self._on_pc, qos)

        # ---- session 目录 ----
        if self.get_parameter('auto_session').value:
            sess = self.get_parameter('session_name').value
            if not sess:
                sess = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._save_dir = os.path.join(
                self.get_parameter('save_dir').value, sess)
        else:
            self._save_dir = self.get_parameter('save_dir').value
        os.makedirs(self._save_dir, exist_ok=True)

        self._count = 0

        # ---- 终端按键监听（独立线程）----
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'key_save running')
        self.get_logger().info(f'  save_dir = {self._save_dir}')
        self.get_logger().info(f'  image    = {self.get_parameter("camera_topic").value}')
        self.get_logger().info(f'  lidar    = {self.get_parameter("lidar_topic").value}')
        self.get_logger().info('  按键: [s/Enter/Space] 保存  [c] 计数  [q] 退出  [h] 帮助')
        self.get_logger().info('=' * 60)
        self._print_status()

    # ---- ROS 回调 ----
    def _on_image(self, msg: Image):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, self.get_parameter('image_encoding').value)
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._latest_image = (img, ts)

    def _on_pc(self, msg: PointCloud2):
        try:
            # 读 x,y,z,(intensity)
            field_names = [f.name for f in msg.fields]
            want = ['x', 'y', 'z']
            if 'intensity' in field_names:
                want.append('intensity')
            pts = np.array(
                list(pc2.read_points(msg, field_names=want, skip_nans=True)),
                dtype=np.float64,
            )
        except Exception as e:
            self.get_logger().warn(f'pointcloud2 read failed: {e}')
            return
        if pts.size == 0:
            return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._latest_pc = (pts, ts)

    # ---- 按键监听 ----
    def _key_loop(self):
        """读 stdin 一个字节，非阻塞 + 阻塞混合。"""
        if not sys.stdin.isatty():
            # 没有 TTY（比如 docker exec 没有 -it），用 /dev/tty 兜底
            try:
                fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            except Exception as e:
                self.get_logger().warn(f'没有可用的 TTY ({e})，按键监听已禁用。'
                                       f'可以发 topic /key_save/trigger std_msgs/Bool 来触发保存。')
                self._key_disabled = True
                self._subscribe_trigger()
                return
        else:
            fd = sys.stdin.fileno()

        self._key_disabled = False
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)   # cbreak 模式：按一个键立刻读到，不等回车
            while rclpy.ok():
                r, _, _ = select.select([fd], [], [], 0.2)
                if r:
                    ch = os.read(fd, 1).decode('utf-8', errors='ignore')
                    self._handle_key(ch)
        except Exception as e:
            self.get_logger().error(f'key_loop error: {e}')
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def _subscribe_trigger(self):
        """TTY 不可用时，开一个 trigger 话题订阅。"""
        from std_msgs.msg import Bool
        self.create_subscription(Bool, '/key_save/trigger',
                                 lambda msg: self._handle_key('s') if msg.data else None, 10)
        self.get_logger().info('已订阅 /key_save/trigger (std_msgs/Bool)，发 true 触发保存')

    def _handle_key(self, ch: str):
        ch_lower = ch.lower()
        if ch_lower in ('s', ' ', '\n', '\r'):
            self._do_save()
        elif ch_lower == 'c':
            self.get_logger().info(f'已保存 {self._count} 组')
        elif ch_lower == 'q':
            self.get_logger().info('退出')
            rclpy.shutdown()
        elif ch_lower == 'h' or ch == '?':
            self.get_logger().info('按键: s/Space/Enter=保存  c=计数  q=退出  h=帮助')
        else:
            pass

    # ---- 保存逻辑 ----
    def _do_save(self):
        with self._lock:
            img_pkg = self._latest_image
            pc_pkg  = self._latest_pc

        if img_pkg is None or pc_pkg is None:
            self.get_logger().warn('没有图像/点云数据可保存（先启动相机和雷达）')
            return

        img, img_ts = img_pkg
        pts, pc_ts  = pc_pkg
        now = _now_ns() * 1e-9
        age_img = now - img_ts
        age_pc  = now - pc_ts
        dt_ms   = abs(img_ts - pc_ts) * 1000.0

        max_age = self.get_parameter('max_latest_age_s').value
        max_dt  = self.get_parameter('max_dt_ms').value
        warn = []
        if age_img > max_age:
            warn.append(f'图像已 {age_img*1000:.0f}ms 旧')
        if age_pc > max_age:
            warn.append(f'点云已 {age_pc*1000:.0f}ms 旧')
        if dt_ms > max_dt:
            warn.append(f'配对 dt={dt_ms:.1f}ms 超过阈值 {max_dt}ms')

        seq = self._count
        sub = os.path.join(self._save_dir, f'{seq:06d}')
        os.makedirs(sub, exist_ok=True)
        img_path  = os.path.join(sub, f'image_{seq:06d}.png')
        pcd_path  = os.path.join(sub, f'points_{seq:06d}.pcd')
        stamp_path = os.path.join(sub, f'stamp_{seq:06d}.txt')

        try:
            cv2.imwrite(img_path, img)
            _save_pcd(pcd_path, pts)
            with open(stamp_path, 'w') as f:
                f.write(f'image_ts = {img_ts:.9f}\n')
                f.write(f'lidar_ts = {pc_ts:.9f}\n')
                f.write(f'dt_ms    = {dt_ms:.3f}\n')
                f.write(f'save_ts  = {now:.9f}\n')
        except Exception as e:
            self.get_logger().error(f'保存失败: {e}')
            return

        self._count += 1
        msg = f'[SAVE {seq:06d}] img={img.shape} pc={pts.shape[0]}pts dt={dt_ms:.1f}ms  -> {sub}'
        if warn:
            msg += '  ⚠ ' + '; '.join(warn)
        self.get_logger().info(msg)

    def _print_status(self):
        # 启动后定时打印缓存状态
        self._status_timer = self.create_timer(2.0, self._print_status_cb)

    def _print_status_cb(self):
        with self._lock:
            img_pkg = self._latest_image
            pc_pkg  = self._latest_pc
        now = time.time()
        if img_pkg is None and pc_pkg is None:
            self.get_logger().info('等待数据…（确保相机/雷达节点已启动）', throttle_duration_sec=10.0)
            return
        line = []
        if img_pkg:
            line.append(f'img age={(now - img_pkg[1])*1000:.0f}ms')
        if pc_pkg:
            line.append(f'pc  age={(now - pc_pkg[1])*1000:.0f}ms')
        if img_pkg and pc_pkg:
            line.append(f'dt={abs(img_pkg[1]-pc_pkg[1])*1000:.1f}ms')
        line.append(f'saved={self._count}')
        self.get_logger().info(' | '.join(line), throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = KeySave()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
