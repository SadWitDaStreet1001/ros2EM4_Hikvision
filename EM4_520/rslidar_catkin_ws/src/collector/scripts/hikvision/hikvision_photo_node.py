#!/usr/bin/env python3

import csv
import os
import threading
import time
from datetime import datetime
from typing import Optional
from urllib.parse import urlsplit, urlunsplit

import cv2
import rospy
import rospkg


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
    return urlunsplit((parts.scheme, "{}@{}".format(userinfo, host), parts.path, parts.query, parts.fragment))


class HikvisionPhotoNode:
    def __init__(self):
        package_path = rospkg.RosPack().get_path("collector")
        default_save_dir = os.path.join(package_path, "data", "hikvision", "photos")

        self.rtsp_url = rospy.get_param(
            "~rtsp_url",
            "rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101",
        )
        self.capture_backend = rospy.get_param("~capture_backend", "ffmpeg").lower()
        self.ffmpeg_capture_options = rospy.get_param(
            "~ffmpeg_capture_options",
            "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0|stimeout;5000000",
        )
        self.save_dir = rospy.get_param("~save_dir", default_save_dir)
        self.save_image_ext = rospy.get_param("~save_image_ext", "png")
        self.preview_scale = float(rospy.get_param("~preview_scale", 0.2))
        self.display_fps = max(0.0, float(rospy.get_param("~display_fps", 10.0)))
        self.reconnect_delay = max(0.1, float(rospy.get_param("~reconnect_delay", 2.0)))
        self.read_fail_limit = max(1, int(rospy.get_param("~read_fail_limit", 25)))
        self.window_name = rospy.get_param("~window_name", "hikvision photo")
        self.frame_wait_timeout = max(0.1, float(rospy.get_param("~frame_wait_timeout", 2.0)))

        self.capture: Optional[cv2.VideoCapture] = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = 0.0
        self.reader_stop = threading.Event()
        self.reader_thread: Optional[threading.Thread] = None
        self.saved_count = 0

        os.makedirs(self.save_dir, exist_ok=True)

        rospy.loginfo("hikvision_photo_node started")
        rospy.loginfo("rtsp_url=%s", sanitize_url(self.rtsp_url))
        rospy.loginfo("save_dir=%s", self.save_dir)
        rospy.loginfo("display=%s", os.environ.get("DISPLAY", ""))
        rospy.loginfo("preview_scale=%.3f display_fps=%.2f", self.preview_scale, self.display_fps)
        rospy.loginfo("time source=local system clock, no lidar sync in photo mode")
        rospy.loginfo("press s to save current frame, q to quit")

    def configure_ffmpeg_options(self):
        if self.capture_backend == "ffmpeg" and self.ffmpeg_capture_options:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = self.ffmpeg_capture_options
            rospy.loginfo("OPENCV_FFMPEG_CAPTURE_OPTIONS=%s", self.ffmpeg_capture_options)

    def open_capture(self) -> bool:
        self.close_capture()
        self.configure_ffmpeg_options()

        if self.capture_backend == "ffmpeg" and hasattr(cv2, "CAP_FFMPEG"):
            capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        elif self.capture_backend == "gstreamer" and hasattr(cv2, "CAP_GSTREAMER"):
            capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_GSTREAMER)
        else:
            capture = cv2.VideoCapture(self.rtsp_url)

        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)

        if not capture.isOpened():
            rospy.logwarn("failed to open RTSP stream: %s", sanitize_url(self.rtsp_url))
            capture.release()
            return False

        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = capture.get(cv2.CAP_PROP_FPS)
        rospy.loginfo("opened RTSP stream: %dx%d %.2f fps", width, height, fps)
        self.capture = capture
        return True

    def close_capture(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def start_reader(self):
        if self.reader_thread is not None and self.reader_thread.is_alive():
            return

        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self.read_latest_frames, daemon=True)
        self.reader_thread.start()

    def stop_reader(self):
        self.reader_stop.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=2.0)
            self.reader_thread = None
        self.close_capture()

    def read_latest_frames(self):
        fail_count = 0

        while not rospy.is_shutdown() and not self.reader_stop.is_set():
            if self.capture is None and not self.open_capture():
                self.reader_stop.wait(self.reconnect_delay)
                continue

            ok, frame = self.capture.read()
            stamp = time.time()
            if not ok or frame is None or frame.size == 0:
                fail_count += 1
                rospy.logwarn_throttle(2.0, "failed to read RTSP frame (%d/%d)", fail_count, self.read_fail_limit)
                if fail_count >= self.read_fail_limit:
                    rospy.logwarn("reconnecting RTSP stream after repeated read failures")
                    self.close_capture()
                    fail_count = 0
                    self.reader_stop.wait(self.reconnect_delay)
                continue

            fail_count = 0
            with self.frame_lock:
                self.latest_frame = frame
                self.latest_stamp = stamp

    def get_latest_frame(self):
        with self.frame_lock:
            return self.latest_frame, self.latest_stamp

    def make_preview(self, frame, stamp_text, age_ms):
        preview = frame
        if 0.0 < self.preview_scale < 1.0:
            preview = cv2.resize(
                frame,
                None,
                fx=self.preview_scale,
                fy=self.preview_scale,
                interpolation=cv2.INTER_AREA,
            )
        preview = preview.copy()
        cv2.rectangle(preview, (0, 0), (preview.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(
            preview,
            "press s to save, q to quit | {} | age {:.0f} ms".format(stamp_text, age_ms),
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (30, 180, 30),
            2,
            cv2.LINE_AA,
        )
        return preview

    def save_frame(self, frame, stamp):
        date_dir = datetime.fromtimestamp(stamp).strftime("%Y%m%d")
        output_dir = os.path.join(self.save_dir, date_dir)
        os.makedirs(output_dir, exist_ok=True)

        stamp_text = "{:.6f}".format(stamp).replace(".", "_")
        image_name = "hikvision_{}.{}".format(stamp_text, self.save_image_ext)
        image_path = os.path.join(output_dir, image_name)

        if not cv2.imwrite(image_path, frame):
            rospy.logerr("failed to write image: %s", image_path)
            return

        metadata_path = os.path.join(output_dir, "photos.csv")
        write_header = not os.path.exists(metadata_path)
        with open(metadata_path, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(["stamp", "image_file", "width", "height", "rtsp_url"])
            writer.writerow(["{:.9f}".format(stamp), image_name, frame.shape[1], frame.shape[0], sanitize_url(self.rtsp_url)])

        self.saved_count += 1
        rospy.loginfo("saved image %s", image_path)

    def spin(self):
        self.start_reader()
        last_stamp = 0.0
        last_display_time = 0.0
        last_wait_log = time.monotonic()
        min_display_period = 1.0 / self.display_fps if self.display_fps > 0.0 else 0.0

        try:
            while not rospy.is_shutdown():
                frame, stamp = self.get_latest_frame()
                if frame is None:
                    if time.monotonic() - last_wait_log >= self.frame_wait_timeout:
                        rospy.logwarn("waiting for RTSP frames from %s", sanitize_url(self.rtsp_url))
                        last_wait_log = time.monotonic()
                    key = cv2.waitKey(10) & 0xFF
                    if key == ord("q"):
                        rospy.signal_shutdown("hikvision photo window closed")
                    continue

                now = time.monotonic()
                should_display = stamp != last_stamp
                if should_display and min_display_period > 0.0:
                    should_display = now - last_display_time >= min_display_period

                if not should_display:
                    key = cv2.waitKey(5) & 0xFF
                    if key == ord("s"):
                        self.save_frame(frame, stamp)
                    elif key == ord("q"):
                        rospy.signal_shutdown("hikvision photo window closed")
                    continue

                last_stamp = stamp
                last_display_time = now
                stamp_text = "{:.6f}".format(stamp)
                age_ms = max(0.0, (time.time() - stamp) * 1000.0)
                cv2.imshow(self.window_name, self.make_preview(frame, stamp_text, age_ms))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    self.save_frame(frame, stamp)
                elif key == ord("q"):
                    rospy.signal_shutdown("hikvision photo window closed")
        finally:
            self.stop_reader()
            cv2.destroyAllWindows()


def main():
    rospy.init_node("hikvision_photo_node")
    node = HikvisionPhotoNode()
    node.spin()


if __name__ == "__main__":
    main()
