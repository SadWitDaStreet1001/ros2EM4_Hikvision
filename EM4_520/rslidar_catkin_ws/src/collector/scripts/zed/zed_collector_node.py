#!/usr/bin/env python3

import csv
import math
import os
import re
import threading

import cv2
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2


class ZedCollectorNode:
    def __init__(self):
        package_path = rospkg.RosPack().get_path("collector")
        default_data_dir = os.path.join(package_path, "data", "zed")

        self.image_topic = rospy.get_param("~image_topic", "/zed/image_raw")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/rslidar_points")
        self.data_dir = rospy.get_param("~data_dir", default_data_dir)
        self.max_sync_age = float(rospy.get_param("~max_sync_age", 0.12))
        self.max_display_points = int(rospy.get_param("~max_display_points", 40000))
        self.display_range_m = float(rospy.get_param("~display_range_m", 20.0))
        self.save_image_ext = rospy.get_param("~save_image_ext", "png")
        self.require_intensity = self._get_bool_param("~require_intensity", True)
        self.require_nonzero_intensity = self._get_bool_param("~require_nonzero_intensity", True)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_cloud = None
        self.latest_cloud_recv_time = None
        self.latest_pair = None
        self.warned_no_intensity = False

        os.makedirs(self.data_dir, exist_ok=True)
        self.saved_count = self._initial_saved_count()
        self.metadata_path = os.path.join(self.data_dir, "index.csv")
        self._init_metadata()

        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=3, buff_size=8 * 1024 * 1024
        )
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=3)

        rospy.loginfo("zed_collector_node subscribing image=%s cloud=%s", self.image_topic, self.cloud_topic)
        rospy.loginfo("zed_collector_node saving data to %s", self.data_dir)

    def _get_bool_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _init_metadata(self):
        if os.path.exists(self.metadata_path):
            return
        with open(self.metadata_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "index",
                "image_stamp",
                "cloud_stamp",
                "image_file",
                "pcd_file",
                "image_topic",
                "cloud_topic",
                "arrival_sync_age_sec",
                "min_intensity",
                "max_intensity",
                "nonzero_intensity_points",
            ])

    def _initial_saved_count(self):
        pattern = re.compile(r"^frame_(\d{6})_.*\.pcd$")
        max_index = 0
        for name in os.listdir(self.data_dir):
            match = pattern.match(name)
            if match:
                max_index = max(max_index, int(match.group(1)))
        return max_index

    def _cloud_callback(self, msg):
        with self.lock:
            self.latest_cloud = msg
            self.latest_cloud_recv_time = rospy.Time.now()

    def _image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "failed to convert image: %s", exc)
            return

        with self.lock:
            cloud = self.latest_cloud
            cloud_recv_time = self.latest_cloud_recv_time

        if cloud is None or cloud_recv_time is None:
            self._show_waiting(image, "waiting for point cloud")
            return

        image_recv_time = rospy.Time.now()
        sync_age = abs((image_recv_time - cloud_recv_time).to_sec())
        if sync_age > self.max_sync_age:
            status = "cloud too old/new: {:.3f}s".format(sync_age)
            self._show_waiting(image, status)
            return

        cloud_view = self._render_cloud_view(cloud)
        self._draw_status(image, "synced {:.3f}s | press s to save, q to quit".format(sync_age), (30, 180, 30))
        self._draw_status(cloud_view, "points: {} | stamp: {:.6f}".format(cloud.width * cloud.height, cloud.header.stamp.to_sec()), (30, 180, 30))

        with self.lock:
            self.latest_pair = {
                "image": image.copy(),
                "image_stamp": msg.header.stamp,
                "cloud": cloud,
                "cloud_stamp": cloud.header.stamp,
                "sync_age": sync_age,
            }

        cv2.imshow("zed synchronized image", image)
        cv2.imshow("zed lidar top view", cloud_view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            self._save_latest_pair()
        elif key == ord("q"):
            rospy.signal_shutdown("collector window closed")

    def _show_waiting(self, image, status):
        self._draw_status(image, status, (20, 20, 220))
        cloud_view = np.zeros((700, 700, 3), dtype=np.uint8)
        self._draw_status(cloud_view, status, (20, 20, 220))
        cv2.imshow("zed synchronized image", image)
        cv2.imshow("zed lidar top view", cloud_view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rospy.signal_shutdown("collector window closed")

    def _draw_status(self, image, text, color):
        cv2.rectangle(image, (0, 0), (image.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(image, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)

    def _render_cloud_view(self, cloud):
        canvas_size = 700
        canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        center = canvas_size // 2
        scale = (canvas_size * 0.46) / max(self.display_range_m, 0.1)

        intensity_field = self._find_field_name(cloud, "intensity")
        field_names = ("x", "y", "z", intensity_field) if intensity_field else ("x", "y", "z")
        points_iter = point_cloud2.read_points(cloud, field_names=field_names, skip_nans=True)

        points = []
        for i, point in enumerate(points_iter):
            if i >= self.max_display_points:
                break
            x = float(point[0])
            y = float(point[1])
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            intensity = float(point[3]) if intensity_field else 0.0
            points.append((x, y, intensity))

        if points:
            arr = np.asarray(points, dtype=np.float32)
            px = (center - arr[:, 1] * scale).astype(np.int32)
            py = (center - arr[:, 0] * scale).astype(np.int32)
            valid = (px >= 0) & (px < canvas_size) & (py >= 0) & (py < canvas_size)
            if np.any(valid):
                intensities = np.clip(arr[:, 2], 0, 255).astype(np.uint8)
                color_map = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
                colors = cv2.applyColorMap(intensities, color_map).reshape(-1, 3)
                canvas[py[valid], px[valid]] = colors[valid]

        cv2.line(canvas, (center, 0), (center, canvas_size), (50, 50, 50), 1)
        cv2.line(canvas, (0, center), (canvas_size, center), (50, 50, 50), 1)
        cv2.circle(canvas, (center, center), 4, (255, 255, 255), -1)
        return canvas

    def _save_latest_pair(self):
        with self.lock:
            pair = self.latest_pair

        if pair is None:
            rospy.logwarn("no synchronized image/cloud pair to save")
            return

        save_index = self.saved_count + 1
        stamp = pair["image_stamp"].to_sec()
        stem = "frame_{:06d}_{:.6f}".format(save_index, stamp)
        image_name = "{}.{}".format(stem, self.save_image_ext)
        pcd_name = "{}.pcd".format(stem)
        image_path = os.path.join(self.data_dir, image_name)
        pcd_path = os.path.join(self.data_dir, pcd_name)

        pcd_stats = self._write_pcd_ascii(pcd_path, pair["cloud"])
        if pcd_stats is None:
            return

        if not cv2.imwrite(image_path, pair["image"]):
            rospy.logerr("failed to write image: %s", image_path)
            if os.path.exists(pcd_path):
                os.remove(pcd_path)
            return

        with open(self.metadata_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                save_index,
                "{:.9f}".format(stamp),
                "{:.9f}".format(pair["cloud_stamp"].to_sec()),
                image_name,
                pcd_name,
                self.image_topic,
                self.cloud_topic,
                "{:.9f}".format(pair["sync_age"]),
                "{:.6f}".format(pcd_stats["min_intensity"]),
                "{:.6f}".format(pcd_stats["max_intensity"]),
                pcd_stats["nonzero_intensity_points"],
            ])

        self.saved_count = save_index

        rospy.loginfo(
            "saved %s and %s (%d points, intensity min/max %.1f/%.1f, nonzero %d)",
            image_name,
            pcd_name,
            pcd_stats["point_count"],
            pcd_stats["min_intensity"],
            pcd_stats["max_intensity"],
            pcd_stats["nonzero_intensity_points"],
        )

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
            message = "PointCloud2 has no intensity field; refusing to save PCD. fields=[{}]".format(field_list)
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
                "Check /rslidar_points intensity values before ZED collection."
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

    def spin(self):
        rospy.spin()

    def close(self):
        cv2.destroyAllWindows()


def main():
    rospy.init_node("zed_collector_node")
    node = ZedCollectorNode()
    rospy.on_shutdown(node.close)
    node.spin()


if __name__ == "__main__":
    main()
