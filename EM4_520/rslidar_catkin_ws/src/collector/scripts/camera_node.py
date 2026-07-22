#!/usr/bin/env python3

import csv
import math
import os
import re
import threading

import cv2
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2


class StereoCameraPublisher:
    def __init__(self):
        package_path = rospkg.RosPack().get_path("collector")
        default_data_dir = os.path.join(package_path, "data")

        self.topic = rospy.get_param("~image_topic", "/collector/camera/image_raw")
        self.left_topic = rospy.get_param("~left_image_topic", "/collector/camera/left/image_raw")
        self.right_topic = rospy.get_param("~right_image_topic", "/collector/camera/right/image_raw")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.left_frame_id = rospy.get_param("~left_frame_id", "camera_left")
        self.right_frame_id = rospy.get_param("~right_frame_id", "camera_right")
        self.camera_index = int(rospy.get_param("~camera_index", 0))
        self.video_device = rospy.get_param("~video_device", "")
        self.width = int(rospy.get_param("~width", 3840))
        self.height = int(rospy.get_param("~height", 1080))
        self.fps = float(rospy.get_param("~fps", 30.0))
        self.show_preview = bool(rospy.get_param("~show_preview", True))
        self.preview_scale = float(rospy.get_param("~preview_scale", 0.5))
        self.save_dir = rospy.get_param("~save_dir", default_data_dir)
        self.save_image_ext = rospy.get_param("~save_image_ext", "png")
        self.save_combined = bool(rospy.get_param("~save_combined", True))
        self.cloud_topic = rospy.get_param("~cloud_topic", "/rslidar_points")
        self.require_cloud = bool(rospy.get_param("~require_cloud", True))
        self.require_intensity = bool(rospy.get_param("~require_intensity", True))
        self.require_nonzero_intensity = bool(rospy.get_param("~require_nonzero_intensity", True))
        self.max_cloud_age = float(rospy.get_param("~max_cloud_age", 0.2))

        os.makedirs(self.save_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.topic, Image, queue_size=5)
        self.left_pub = rospy.Publisher(self.left_topic, Image, queue_size=5)
        self.right_pub = rospy.Publisher(self.right_topic, Image, queue_size=5)
        self.stop_event = threading.Event()
        self.last_frame = None
        self.lock = threading.Lock()
        self.latest_cloud = None
        self.latest_cloud_recv_time = None
        self.warned_no_intensity = False
        self.saved_count = self._initial_saved_count()
        self.metadata_path = os.path.join(self.save_dir, "stereo_index.csv")
        self._init_metadata()
        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=3, buff_size=8 * 1024 * 1024
        )

        source = self.video_device if self.video_device else self.camera_index
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError("failed to open camera source: {}".format(source))

        if self.width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo(
            "camera_node source=%s requested=%dx%d actual=%dx%d fps=%.1f cloud=%s",
            source,
            self.width,
            self.height,
            actual_w,
            actual_h,
            self.fps,
            self.cloud_topic,
        )
        if actual_w % 2 != 0:
            rospy.logwarn("camera width %d is odd; stereo split may be invalid", actual_w)

    def _initial_saved_count(self):
        pattern = re.compile(r"^stereo_(\d{6})_.*_left\..+$")
        max_index = 0
        for name in os.listdir(self.save_dir):
            match = pattern.match(name)
            if match:
                max_index = max(max_index, int(match.group(1)))
        return max_index

    def _init_metadata(self):
        if os.path.exists(self.metadata_path):
            return
        with open(self.metadata_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "index",
                    "stamp",
                    "folder",
                    "left_file",
                    "right_file",
                    "combined_file",
                    "pcd_file",
                    "width",
                    "height",
                    "left_width",
                    "left_height",
                    "right_width",
                    "right_height",
                    "cloud_stamp",
                    "cloud_age_sec",
                    "point_count",
                    "min_intensity",
                    "max_intensity",
                    "nonzero_intensity_points",
                    "image_topic",
                    "left_image_topic",
                    "right_image_topic",
                    "cloud_topic",
                    "source",
                ]
            )

    def _split_stereo(self, frame):
        height, width = frame.shape[:2]
        if width < 2 or width % 2 != 0:
            raise RuntimeError("invalid stereo frame size: {}x{}".format(width, height))
        half = width // 2
        left = frame[:, :half].copy()
        right = frame[:, half:].copy()
        return left, right

    def _make_preview(self, left, right):
        preview = cv2.hconcat([left, right])
        cv2.rectangle(preview, (0, 0), (preview.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(
            preview,
            "stereo preview | press s to save left/right, q to quit",
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (30, 180, 30),
            2,
            cv2.LINE_AA,
        )
        if 0.0 < self.preview_scale < 1.0:
            preview = cv2.resize(
                preview,
                None,
                fx=self.preview_scale,
                fy=self.preview_scale,
                interpolation=cv2.INTER_AREA,
            )
        return preview

    def _cloud_callback(self, msg):
        with self.lock:
            self.latest_cloud = msg
            self.latest_cloud_recv_time = rospy.Time.now()

    def _get_cloud_for_save(self):
        with self.lock:
            cloud = self.latest_cloud
            cloud_recv_time = self.latest_cloud_recv_time

        if cloud is None or cloud_recv_time is None:
            if self.require_cloud:
                rospy.logerr("no point cloud available yet on %s", self.cloud_topic)
            return None, None

        cloud_age = abs((rospy.Time.now() - cloud_recv_time).to_sec())
        if self.require_cloud and cloud_age > self.max_cloud_age:
            rospy.logerr(
                "latest point cloud is too old/new: %.3fs > %.3fs on %s",
                cloud_age,
                self.max_cloud_age,
                self.cloud_topic,
            )
            return None, None

        return cloud, cloud_age

    def _find_field_name(self, cloud, expected_name):
        for field in cloud.fields:
            if field.name == expected_name:
                return field.name
        for field in cloud.fields:
            if field.name.lower() == expected_name.lower():
                return field.name
        return None

    def _write_pcd_ascii(self, path, cloud):
        intensity_field = self._find_field_name(cloud, "intensity")
        if intensity_field is None:
            field_list = ", ".join(field.name for field in cloud.fields)
            message = "PointCloud2 has no intensity field; fields=[{}]".format(field_list)
            if self.require_intensity:
                rospy.logerr(message)
                return None
            if not self.warned_no_intensity:
                rospy.logwarn("%s; saved intensity will be 0 because require_intensity is false", message)
                self.warned_no_intensity = True

        field_names = ("x", "y", "z", intensity_field) if intensity_field else ("x", "y", "z")
        points = []
        min_intensity = float("inf")
        max_intensity = float("-inf")
        nonzero_intensity_points = 0

        for point in point_cloud2.read_points(cloud, field_names=field_names, skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            intensity = float(point[3]) if intensity_field else 0.0
            if all(math.isfinite(v) for v in (x, y, z, intensity)):
                points.append((x, y, z, intensity))
                min_intensity = min(min_intensity, intensity)
                max_intensity = max(max_intensity, intensity)
                if abs(intensity) > 1e-6:
                    nonzero_intensity_points += 1

        if not points:
            rospy.logerr("no valid points to save; refusing to write PCD")
            return None

        if intensity_field and self.require_nonzero_intensity and nonzero_intensity_points == 0:
            rospy.logerr(
                "all valid points have zero intensity; refusing to save PCD. "
                "Check /rslidar_points intensity values before collection."
            )
            return None

        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write("WIDTH {}\n".format(len(points)))
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS {}\n".format(len(points)))
            f.write("DATA ascii\n")
            for x, y, z, intensity in points:
                f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(x, y, z, intensity))

        return {
            "point_count": len(points),
            "min_intensity": min_intensity,
            "max_intensity": max_intensity,
            "nonzero_intensity_points": nonzero_intensity_points,
        }

    def _save_current_frame(self):
        if self.last_frame is None:
            rospy.logwarn("no stereo frame available yet")
            return

        cloud, cloud_age = self._get_cloud_for_save()
        if self.require_cloud and cloud is None:
            return

        stamp = self.last_frame["stamp"].to_sec()
        self.saved_count += 1
        folder_name = "capture_{:06d}_{:.6f}".format(self.saved_count, stamp)
        capture_dir = os.path.join(self.save_dir, folder_name)
        os.makedirs(capture_dir, exist_ok=True)

        stem = "stereo_{:06d}_{:.6f}".format(self.saved_count, stamp)

        left_name = "{}_left.{}".format(stem, self.save_image_ext)
        right_name = "{}_right.{}".format(stem, self.save_image_ext)
        combined_name = "{}_combined.{}".format(stem, self.save_image_ext) if self.save_combined else ""
        pcd_name = "{}.pcd".format(stem)

        left_path = os.path.join(capture_dir, left_name)
        right_path = os.path.join(capture_dir, right_name)
        combined_path = os.path.join(capture_dir, combined_name) if combined_name else ""
        pcd_path = os.path.join(capture_dir, pcd_name)

        if not cv2.imwrite(left_path, self.last_frame["left"]):
            rospy.logerr("failed to save left image: %s", left_path)
            self.saved_count -= 1
            os.rmdir(capture_dir)
            return
        if not cv2.imwrite(right_path, self.last_frame["right"]):
            rospy.logerr("failed to save right image: %s", right_path)
            if os.path.exists(left_path):
                os.remove(left_path)
            self.saved_count -= 1
            os.rmdir(capture_dir)
            return
        if combined_name and not cv2.imwrite(combined_path, self.last_frame["combined"]):
            rospy.logwarn("failed to save combined image: %s", combined_path)
            combined_name = ""
            combined_path = ""

        pcd_stats = None
        cloud_stamp = ""
        if cloud is not None:
            pcd_stats = self._write_pcd_ascii(pcd_path, cloud)
            if pcd_stats is None:
                for path in (left_path, right_path, combined_path):
                    if path and os.path.exists(path):
                        os.remove(path)
                self.saved_count -= 1
                if os.path.isdir(capture_dir) and not os.listdir(capture_dir):
                    os.rmdir(capture_dir)
                return
            cloud_stamp = "{:.9f}".format(cloud.header.stamp.to_sec())
        else:
            pcd_name = ""
            cloud_age = ""

        with open(self.metadata_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    self.saved_count,
                    "{:.9f}".format(stamp),
                    folder_name,
                    left_name,
                    right_name,
                    combined_name,
                    pcd_name,
                    self.last_frame["combined"].shape[1],
                    self.last_frame["combined"].shape[0],
                    self.last_frame["left"].shape[1],
                    self.last_frame["left"].shape[0],
                    self.last_frame["right"].shape[1],
                    self.last_frame["right"].shape[0],
                    cloud_stamp,
                    "" if cloud_age == "" else "{:.9f}".format(cloud_age),
                    "" if pcd_stats is None else pcd_stats["point_count"],
                    "" if pcd_stats is None else "{:.6f}".format(pcd_stats["min_intensity"]),
                    "" if pcd_stats is None else "{:.6f}".format(pcd_stats["max_intensity"]),
                    "" if pcd_stats is None else pcd_stats["nonzero_intensity_points"],
                    self.topic,
                    self.left_topic,
                    self.right_topic,
                    self.cloud_topic,
                    self.video_device if self.video_device else str(self.camera_index),
                ]
            )

        rospy.loginfo("saved stereo pair: %s, %s%s",
                      os.path.join(folder_name, left_name),
                      os.path.join(folder_name, right_name),
                      "" if not combined_name else ", " + os.path.join(folder_name, combined_name))
        if pcd_stats is not None:
            rospy.loginfo(
                "saved point cloud: %s (%d points, intensity min/max %.1f/%.1f, nonzero %d)",
                os.path.join(folder_name, pcd_name),
                pcd_stats["point_count"],
                pcd_stats["min_intensity"],
                pcd_stats["max_intensity"],
                pcd_stats["nonzero_intensity_points"],
            )

    def spin(self):
        rate_hz = self.fps if self.fps > 0 else 30.0
        rate = rospy.Rate(rate_hz)
        rospy.loginfo(
            "camera_node publishing %s, %s, %s and saving cloud from %s",
            self.topic,
            self.left_topic,
            self.right_topic,
            self.cloud_topic,
        )

        while not rospy.is_shutdown() and not self.stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok:
                rospy.logwarn_throttle(2.0, "camera frame grab failed")
                rate.sleep()
                continue

            try:
                left, right = self._split_stereo(frame)
            except RuntimeError as exc:
                rospy.logerr_throttle(2.0, "%s", exc)
                rate.sleep()
                continue

            stamp = rospy.Time.now()
            combined_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            combined_msg.header.stamp = stamp
            combined_msg.header.frame_id = self.frame_id
            self.pub.publish(combined_msg)

            left_msg = self.bridge.cv2_to_imgmsg(left, encoding="bgr8")
            left_msg.header.stamp = stamp
            left_msg.header.frame_id = self.left_frame_id
            self.left_pub.publish(left_msg)

            right_msg = self.bridge.cv2_to_imgmsg(right, encoding="bgr8")
            right_msg.header.stamp = stamp
            right_msg.header.frame_id = self.right_frame_id
            self.right_pub.publish(right_msg)

            self.last_frame = {
                "stamp": stamp,
                "combined": frame.copy(),
                "left": left,
                "right": right,
            }

            if self.show_preview:
                preview = self._make_preview(left, right)
                cv2.imshow("collector stereo preview", preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    self._save_current_frame()
                elif key == ord("q"):
                    rospy.signal_shutdown("camera preview closed")

            rate.sleep()

    def close(self):
        self.stop_event.set()
        if self.cap is not None:
            self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("camera_node")
    node = None
    try:
        node = StereoCameraPublisher()
        rospy.on_shutdown(node.close)
        node.spin()
    except Exception as exc:
        rospy.logerr("%s", exc)
        if node is not None:
            node.close()
        raise


if __name__ == "__main__":
    main()
