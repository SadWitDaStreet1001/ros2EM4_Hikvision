"""
Hikvision 网络相机 ROS2 节点。

把 ROS1 的 hikvision_node.py 直接搬到 ROS2 rclpy。
通过 RTSP 拉流（默认 FFMPEG 后端），发 sensor_msgs/Image。
"""
import os
import time
from urllib.parse import urlsplit, urlunsplit

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def sanitize_url(url: str) -> str:
    parts = urlsplit(url)
    if "@" not in parts.netloc:
        return url
    userinfo, host = parts.netloc.rsplit("@", 1)
    if ":" in userinfo:
        username, _ = userinfo.split(":", 1)
        userinfo = "{}:****".format(username)
    else:
        userinfo = "****"
    return urlunsplit((parts.scheme, "{}@{}".format(userinfo, host),
                       parts.path, parts.query, parts.fragment))


class HikvisionNode(Node):
    def __init__(self):
        super().__init__('hikvision_node')

        # ---- 参数（与 ROS1 版完全对齐）----
        self.declare_parameter('rtsp_url', 'rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101')
        self.declare_parameter('image_topic', '/hikvision/image_raw')
        self.declare_parameter('frame_id', 'hikvision_camera')
        self.declare_parameter('encoding', 'bgr8')
        self.declare_parameter('capture_backend', 'ffmpeg')
        self.declare_parameter(
            'ffmpeg_capture_options',
            'rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0|stimeout;5000000'
        )
        self.declare_parameter('reconnect_delay', 2.0)
        self.declare_parameter('read_fail_limit', 25)
        self.declare_parameter('target_fps', 0.0)        # 0 = 不限速
        self.declare_parameter('show_local', False)
        self.declare_parameter('report_interval', 5.0)
        self.declare_parameter('publisher_queue_size', 1)

        self.rtsp_url   = self.get_parameter('rtsp_url').value
        self.image_topic = self.get_parameter('image_topic').value
        self.frame_id   = self.get_parameter('frame_id').value
        self.encoding   = self.get_parameter('encoding').value
        self.capture_backend = self.get_parameter('capture_backend').value.lower()
        self.ffmpeg_capture_options = self.get_parameter('ffmpeg_capture_options').value
        self.reconnect_delay = max(0.1, float(self.get_parameter('reconnect_delay').value))
        self.read_fail_limit = max(1, int(self.get_parameter('read_fail_limit').value))
        self.target_fps = float(self.get_parameter('target_fps').value)
        self.show_local = bool(self.get_parameter('show_local').value)
        self.report_interval = max(1.0, float(self.get_parameter('report_interval').value))
        self.publisher_queue_size = max(1, int(self.get_parameter('publisher_queue_size').value))

        # ---- Publisher（BEST_EFFORT, depth=1）----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.publisher_queue_size,
        )
        self.publisher = self.create_publisher(Image, self.image_topic, qos)
        self.bridge = CvBridge()
        self.capture = None

        self.get_logger().info('hikvision_node started')
        self.get_logger().info(f'rtsp_url={sanitize_url(self.rtsp_url)}')
        self.get_logger().info(
            f'image_topic={self.image_topic} frame_id={self.frame_id} encoding={self.encoding}'
        )
        self.get_logger().info(
            f'capture_backend={self.capture_backend} target_fps={self.target_fps:.2f}'
        )

    def _configure_ffmpeg_options(self):
        if self.capture_backend == 'ffmpeg' and self.ffmpeg_capture_options:
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = self.ffmpeg_capture_options
            self.get_logger().info(f'OPENCV_FFMPEG_CAPTURE_OPTIONS={self.ffmpeg_capture_options}')

    def _open_capture(self) -> bool:
        self._close_capture()
        self._configure_ffmpeg_options()

        if self.capture_backend == 'ffmpeg' and hasattr(cv2, 'CAP_FFMPEG'):
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        elif self.capture_backend == 'gstreamer' and hasattr(cv2, 'CAP_GSTREAMER'):
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_GSTREAMER)
        else:
            cap = cv2.VideoCapture(self.rtsp_url)

        if hasattr(cv2, 'CAP_PROP_BUFFERSIZE'):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().warn(f'failed to open RTSP stream: {sanitize_url(self.rtsp_url)}')
            cap.release()
            return False

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'opened RTSP stream: {w}x{h} {fps:.2f} fps')
        self.capture = cap
        return True

    def _close_capture(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def _publish(self, frame):
        stamp = self.get_clock().now().to_msg()
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def run(self):
        frame_count = 0
        fail_count = 0
        last_report = time.monotonic()
        next_publish = 0.0
        min_period = 1.0 / self.target_fps if self.target_fps > 0 else 0.0

        while rclpy.ok():
            if self.capture is None and not self._open_capture():
                time.sleep(self.reconnect_delay)
                continue

            ok, frame = self.capture.read()
            if not ok or frame is None or frame.size == 0:
                fail_count += 1
                if fail_count % 25 == 0:
                    self.get_logger().warn(
                        f'failed to read RTSP frame ({fail_count}/{self.read_fail_limit})'
                    )
                if fail_count >= self.read_fail_limit:
                    self.get_logger().warn('reconnecting RTSP stream after repeated read failures')
                    self._close_capture()
                    fail_count = 0
                    time.sleep(self.reconnect_delay)
                continue

            fail_count = 0
            now = time.monotonic()
            if min_period > 0.0:
                if next_publish <= 0.0:
                    next_publish = now
                elif now < next_publish:
                    continue

            self._publish(frame)
            if min_period > 0.0:
                next_publish += min_period
                if next_publish < now - min_period:
                    next_publish = now + min_period
            frame_count += 1

            if self.show_local:
                cv2.imshow('hikvision', frame)
                cv2.waitKey(1)

            if now - last_report >= self.report_interval:
                self.get_logger().info(
                    f'published {frame_count / (now - last_report):.2f} fps to {self.image_topic}'
                )
                frame_count = 0
                last_report = now

        self._close_capture()
        if self.show_local:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HikvisionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
