#!/usr/bin/env python3

import csv
import json
import math
import os
import re
import threading
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField


POINT_FIELD_TO_PCD = {
    PointField.INT8: (1, "I"),
    PointField.UINT8: (1, "U"),
    PointField.INT16: (2, "I"),
    PointField.UINT16: (2, "U"),
    PointField.INT32: (4, "I"),
    PointField.UINT32: (4, "U"),
    PointField.FLOAT32: (4, "F"),
    PointField.FLOAT64: (8, "F"),
}


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class HikvisionCollectorNode:
    def __init__(self):
        package_path = rospkg.RosPack().get_path("collector")
        default_data_dir = os.path.join(package_path, "data", "hikvision")

        self.image_topic = rospy.get_param("~image_topic", "/hikvision/image_raw")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/rslidar_points")
        self.data_dir = rospy.get_param("~data_dir", default_data_dir)
        self.max_sync_age = float(rospy.get_param("~max_sync_age", 0.02))
        self.image_buffer_size = max(5, int(rospy.get_param("~image_buffer_size", 10)))
        self.cloud_buffer_size = max(5, int(rospy.get_param("~cloud_buffer_size", 60)))
        self.max_display_points = int(rospy.get_param("~max_display_points", 40000))
        self.display_range_m = float(rospy.get_param("~display_range_m", 20.0))
        self.save_image_ext = rospy.get_param("~save_image_ext", "png")
        self.preview_scale = float(rospy.get_param("~preview_scale", 0.2))
        self.display_fps = max(0.0, float(rospy.get_param("~display_fps", 10.0)))
        self.show_image_preview = get_bool_param("~show_image_preview", True)
        self.show_lidar_preview = get_bool_param("~show_lidar_preview", False)
        self.image_sub_queue_size = max(1, int(rospy.get_param("~image_sub_queue_size", 1)))
        self.cloud_sub_queue_size = max(1, int(rospy.get_param("~cloud_sub_queue_size", 10)))
        self.single_sync_timeout = max(
            self.max_sync_age,
            float(rospy.get_param("~single_sync_timeout", 0.5)),
        )

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.images = deque(maxlen=self.image_buffer_size)
        self.clouds = deque(maxlen=self.cloud_buffer_size)
        self.image_seq = 0
        self.pending_center_seq = None
        self.pending_single_trigger = None
        self.pending_single_cloud = None
        self.saving = False
        self.save_thread = None
        self.last_preview_time = 0.0

        os.makedirs(self.data_dir, exist_ok=True)

        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=self.cloud_sub_queue_size,
            buff_size=16 * 1024 * 1024,
        )
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=self.image_sub_queue_size,
            buff_size=32 * 1024 * 1024,
        )

        rospy.loginfo("hikvision_collector_node subscribing image=%s cloud=%s", self.image_topic, self.cloud_topic)
        rospy.loginfo("hikvision_collector_node saving data to %s", self.data_dir)
        rospy.loginfo("max_sync_age=%.3fs image_buffer=%d cloud_buffer=%d", self.max_sync_age, self.image_buffer_size, self.cloud_buffer_size)
        rospy.loginfo(
            "preview image=%s lidar=%s scale=%.3f display_fps=%.2f image_queue=%d cloud_queue=%d",
            self.show_image_preview,
            self.show_lidar_preview,
            self.preview_scale,
            self.display_fps,
            self.image_sub_queue_size,
            self.cloud_sub_queue_size,
        )
        rospy.loginfo("single_sync_timeout=%.3fs", self.single_sync_timeout)

    def _cloud_callback(self, msg):
        recv_stamp = rospy.Time.now()
        sample = {
            "cloud": msg,
            "recv_stamp": recv_stamp,
            "header_stamp": msg.header.stamp,
        }
        with self.lock:
            self.clouds.append(sample)
        self._attempt_pending_single_save()

    def _image_callback(self, msg):
        recv_stamp = rospy.Time.now()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "failed to convert Hikvision image: %s", exc)
            return

        with self.lock:
            self.image_seq += 1
            sample = {
                "seq": self.image_seq,
                "image": image.copy(),
                "header_stamp": msg.header.stamp,
                "recv_stamp": recv_stamp,
                "frame_id": msg.header.frame_id,
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
            }
            self.images.append(sample)
            latest_cloud = self.clouds[-1] if self.clouds else None
            pending = None
            if self.pending_center_seq is not None:
                pending = "three_frame"
            elif self.pending_single_trigger is not None:
                pending = "single_frame"
            saving = self.saving

        self._show_preview(sample, latest_cloud, pending, saving)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            self._request_save(sample["seq"])
        elif key == ord("d"):
            self._request_single_save(rospy.Time.now())
        elif key == ord("q"):
            rospy.signal_shutdown("hikvision collector window closed")

        self._attempt_pending_save()
        self._attempt_pending_single_save()

    def _request_save(self, center_seq):
        with self.lock:
            if len(self.images) < 2:
                rospy.logwarn("need at least one previous image before saving a three-frame group")
                return
            if self.saving:
                rospy.logwarn("save already in progress")
                return
            if self.pending_single_trigger is not None:
                rospy.logwarn("single-frame save already requested")
                return
            self.pending_center_seq = center_seq
        rospy.loginfo("three-frame save requested at image seq=%d; waiting for next frame", center_seq)

    def _request_single_save(self, trigger_stamp):
        with self.lock:
            if self.saving:
                rospy.logwarn("save already in progress")
                return
            if self.pending_center_seq is not None or self.pending_single_trigger is not None:
                rospy.logwarn("save already requested")
                return
            images = list(self.images)
            clouds = list(self.clouds)

            if not images:
                rospy.logwarn("no Hikvision image available; cannot save current frame")
                return
            if not clouds:
                rospy.logwarn("no point cloud available; cannot save current frame")
                return

            self.pending_single_trigger = trigger_stamp
            self.pending_single_cloud = None

        rospy.loginfo(
            "single-frame software trigger %.6f requested; waiting for next lidar frame",
            trigger_stamp.to_sec(),
        )

    def _attempt_pending_save(self):
        with self.lock:
            if self.saving or self.pending_center_seq is None:
                return

            center_seq = self.pending_center_seq
            images = list(self.images)
            center_index = next((i for i, item in enumerate(images) if item["seq"] == center_seq), None)
            if center_index is None:
                self.pending_center_seq = None
                rospy.logwarn("requested center image seq=%d dropped from buffer before save", center_seq)
                return
            if center_index == 0 or center_index + 1 >= len(images):
                return
            if not self.clouds:
                rospy.logwarn("no point cloud available; keep waiting for cloud data")
                return

            frame_group = [
                ("prev", images[center_index - 1]),
                ("center", images[center_index]),
                ("next", images[center_index + 1]),
            ]
            clouds = list(self.clouds)
            self.pending_center_seq = None

        self._start_save_job("three_frame", frame_group, clouds)

    def _attempt_pending_single_save(self):
        with self.lock:
            if self.saving or self.pending_single_trigger is None:
                return

            now = rospy.Time.now()
            trigger_stamp = self.pending_single_trigger
            images = list(self.images)
            clouds = list(self.clouds)
            if (now - trigger_stamp).to_sec() > self.single_sync_timeout:
                self.pending_single_trigger = None
                self.pending_single_cloud = None
                rospy.logwarn(
                    "timed out waiting for synchronized lidar/image pair after trigger %.6f",
                    trigger_stamp.to_sec(),
                )
                return

            if not images:
                self.pending_single_trigger = None
                self.pending_single_cloud = None
                rospy.logwarn("no Hikvision image available after software trigger %.6f", trigger_stamp.to_sec())
                return
            if not clouds:
                return

            if self.pending_single_cloud is None:
                self.pending_single_cloud = next(
                    (item for item in clouds if item["recv_stamp"] >= trigger_stamp),
                    None,
                )
                if self.pending_single_cloud is None:
                    return

            sync_cloud = self.pending_single_cloud
            sync_target_stamp = sync_cloud["recv_stamp"]
            if (now - sync_target_stamp).to_sec() < self.max_sync_age:
                return

            image_match = self._find_nearest_image(sync_target_stamp, images)
            if image_match is None:
                self.pending_single_trigger = None
                self.pending_single_cloud = None
                rospy.logwarn("no Hikvision image available near lidar frame %.6f", sync_target_stamp.to_sec())
                return

            if image_match["delta_sec"] > self.max_sync_age:
                self.pending_single_trigger = None
                self.pending_single_cloud = None
                rospy.logwarn(
                    "refuse single-frame save: nearest image/lidar delta %.1f ms exceeds strict limit %.1f ms",
                    image_match["delta_sec"] * 1000.0,
                    self.max_sync_age * 1000.0,
                )
                return

            image_sample = dict(image_match["image"])
            image_sample["trigger_delta_sec"] = (image_sample["header_stamp"] - trigger_stamp).to_sec()
            image_sample["sync_target_delta_sec"] = image_match["delta_signed_sec"]
            frame_group = [("current", image_sample)]
            selected_clouds = [sync_cloud]
            self.pending_single_trigger = None
            self.pending_single_cloud = None

        self._start_save_job(
            "single_frame",
            frame_group,
            selected_clouds,
            trigger_stamp=trigger_stamp,
            sync_target_stamp=sync_target_stamp,
            sync_reference="next_lidar_frame",
        )

    def _start_save_job(
        self,
        save_mode,
        frame_group,
        clouds,
        trigger_stamp=None,
        sync_target_stamp=None,
        sync_reference=None,
    ):
        with self.lock:
            if self.saving:
                rospy.logwarn("save already in progress")
                return
            self.saving = True

        thread = threading.Thread(
            target=self._run_save_job,
            args=(save_mode, frame_group, clouds, trigger_stamp, sync_target_stamp, sync_reference),
            daemon=True,
        )
        self.save_thread = thread
        thread.start()
        rospy.loginfo(
            "%s save started in background with %d image(s)",
            save_mode,
            len(frame_group),
        )

    def _run_save_job(self, save_mode, frame_group, clouds, trigger_stamp, sync_target_stamp, sync_reference):
        try:
            self._save_group(
                frame_group,
                clouds,
                save_mode,
                trigger_stamp=trigger_stamp,
                sync_target_stamp=sync_target_stamp,
                sync_reference=sync_reference,
            )
        except Exception as exc:
            rospy.logerr("Hikvision %s save failed: %s", save_mode, exc)
        finally:
            with self.lock:
                self.saving = False

    def _save_group(self, frame_group, clouds, save_mode, trigger_stamp=None, sync_target_stamp=None, sync_reference=None):
        center_index = 1 if save_mode == "three_frame" else 0
        center_image = frame_group[center_index][1]
        single_trigger_mode = save_mode == "single_frame" and trigger_stamp is not None
        center_match_target = sync_target_stamp if sync_target_stamp is not None else center_image["header_stamp"]
        center_cloud_match = self._find_nearest_cloud(center_match_target, clouds)
        if center_cloud_match is None:
            rospy.logwarn("no point cloud available for requested Hikvision group")
            return

        if single_trigger_mode:
            center_cloud_match = dict(center_cloud_match)
            center_cloud_match["cloud_trigger_delta_sec"] = (
                center_cloud_match["match_stamp"] - trigger_stamp
            ).to_sec()
            center_cloud_match["cloud_sync_target_delta_sec"] = (
                center_cloud_match["match_stamp"] - sync_target_stamp
            ).to_sec() if sync_target_stamp is not None else None
            center_cloud_match["delta_sec"] = abs(
                (center_image["header_stamp"] - center_cloud_match["match_stamp"]).to_sec()
            )

        per_frame_matches = []
        fallback = False
        if save_mode == "three_frame":
            for label, image_sample in frame_group:
                match = self._find_nearest_cloud(image_sample["header_stamp"], clouds)
                if match is None or abs(match["delta_sec"]) > self.max_sync_age:
                    fallback = True
                    break
                per_frame_matches.append((label, image_sample, match))

            if fallback:
                rospy.logwarn(
                    "refuse three-frame save: one or more image/cloud deltas exceed strict limit %.1f ms",
                    self.max_sync_age * 1000.0,
                )
                return
        else:
            if abs(center_cloud_match["delta_sec"]) > self.max_sync_age:
                rospy.logwarn(
                    "refuse single-frame save: current image/cloud delta %.1f ms exceeds strict limit %.1f ms",
                    center_cloud_match["delta_sec"] * 1000.0,
                    self.max_sync_age * 1000.0,
                )
                return
            per_frame_matches = [(frame_group[0][0], center_image, center_cloud_match)]

        center_stamp_text = self._stamp_for_name(center_image["header_stamp"])
        date_dir = datetime.fromtimestamp(center_image["header_stamp"].to_sec()).strftime("%Y%m%d")
        prefix = "group" if save_mode == "three_frame" else "single"
        group_dir = os.path.join(self.data_dir, date_dir, "{}_{}".format(prefix, center_stamp_text))
        if os.path.exists(group_dir):
            group_dir = self._dedupe_group_dir(group_dir)
        os.makedirs(group_dir, exist_ok=False)

        rows = []
        metadata = {
            "group": os.path.basename(group_dir),
            "created_wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "image_topic": self.image_topic,
            "cloud_topic": self.cloud_topic,
            "save_mode": save_mode,
            "sync_reference": sync_reference or ("software_trigger" if single_trigger_mode else "image_header_stamp"),
            "trigger_stamp": self._stamp_float(trigger_stamp) if single_trigger_mode else None,
            "sync_target_stamp": self._stamp_float(sync_target_stamp) if sync_target_stamp is not None else None,
            "max_sync_age_sec": self.max_sync_age,
            "used_center_cloud_fallback": fallback,
            "frames": [],
        }

        try:
            for label, image_sample, match in per_frame_matches:
                image_stamp_text = self._stamp_for_name(image_sample["header_stamp"])
                cloud_stamp_text = self._stamp_for_name(match["match_stamp"])
                image_name = "{}_image_{}.{}".format(label, image_stamp_text, self.save_image_ext)
                pcd_name = "{}_cloud_{}.pcd".format(label, cloud_stamp_text)
                image_path = os.path.join(group_dir, image_name)
                pcd_path = os.path.join(group_dir, pcd_name)

                if not cv2.imwrite(image_path, image_sample["image"]):
                    raise RuntimeError("failed to write image: {}".format(image_path))

                pcd_stats = self._write_pcd(pcd_path, match["cloud"])

                item = {
                    "label": label,
                    "image_file": image_name,
                    "pcd_file": pcd_name,
                    "image_header_stamp": self._stamp_float(image_sample["header_stamp"]),
                    "image_recv_stamp": self._stamp_float(image_sample["recv_stamp"]),
                    "cloud_header_stamp": self._stamp_float(match["cloud_header_stamp"]),
                    "cloud_recv_stamp": self._stamp_float(match["cloud_recv_stamp"]),
                    "matched_cloud_stamp": self._stamp_float(match["match_stamp"]),
                    "trigger_stamp": self._stamp_float(trigger_stamp) if single_trigger_mode else None,
                    "sync_target_stamp": self._stamp_float(sync_target_stamp) if sync_target_stamp is not None else None,
                    "image_trigger_delta_sec": (
                        (image_sample["header_stamp"] - trigger_stamp).to_sec()
                        if single_trigger_mode
                        else None
                    ),
                    "image_trigger_delta_ms": (
                        (image_sample["header_stamp"] - trigger_stamp).to_sec() * 1000.0
                        if single_trigger_mode
                        else None
                    ),
                    "cloud_trigger_delta_sec": (
                        match.get("cloud_trigger_delta_sec")
                        if single_trigger_mode
                        else None
                    ),
                    "cloud_trigger_delta_ms": (
                        match.get("cloud_trigger_delta_sec") * 1000.0
                        if single_trigger_mode
                        else None
                    ),
                    "image_sync_target_delta_sec": (
                        (image_sample["header_stamp"] - sync_target_stamp).to_sec()
                        if sync_target_stamp is not None
                        else None
                    ),
                    "image_sync_target_delta_ms": (
                        (image_sample["header_stamp"] - sync_target_stamp).to_sec() * 1000.0
                        if sync_target_stamp is not None
                        else None
                    ),
                    "cloud_sync_target_delta_sec": (
                        match.get("cloud_sync_target_delta_sec")
                        if sync_target_stamp is not None
                        else None
                    ),
                    "cloud_sync_target_delta_ms": (
                        match.get("cloud_sync_target_delta_sec") * 1000.0
                        if sync_target_stamp is not None and match.get("cloud_sync_target_delta_sec") is not None
                        else None
                    ),
                    "sync_delta_sec": match["delta_sec"],
                    "sync_delta_ms": match["delta_sec"] * 1000.0,
                    "point_count": pcd_stats["point_count"],
                    "pcd_fields": pcd_stats["fields"],
                    "pcd_data": pcd_stats["data"],
                    "skipped_fields": pcd_stats["skipped_fields"],
                    "fallback_to_center_cloud": fallback,
                }
                metadata["frames"].append(item)
                rows.append(item)

            self._write_metadata(group_dir, metadata, rows)
        except Exception:
            rospy.logerr("failed to save Hikvision group; partial files are kept in %s", group_dir)
            raise

        rospy.loginfo("saved Hikvision %s: %s", save_mode, group_dir)
        for row in rows:
            rospy.loginfo(
                "%s image=%s cloud=%s sync=%.3f ms points=%d data=%s",
                row["label"],
                row["image_file"],
                row["pcd_file"],
                row["sync_delta_ms"],
                row["point_count"],
                row["pcd_data"],
            )

    def _find_nearest_cloud(self, stamp, clouds):
        if not clouds:
            return None

        best = None
        for sample in clouds:
            match_stamp = sample["recv_stamp"]
            delta_sec = abs((stamp - match_stamp).to_sec())
            if best is None or delta_sec < best["delta_sec"]:
                best = {
                    "cloud": sample["cloud"],
                    "cloud_header_stamp": sample["header_stamp"],
                    "cloud_recv_stamp": sample["recv_stamp"],
                    "match_stamp": match_stamp,
                    "delta_sec": delta_sec,
                }
        return best

    def _find_nearest_image(self, stamp, images):
        if not images:
            return None

        best = None
        for sample in images:
            delta_signed_sec = (sample["header_stamp"] - stamp).to_sec()
            delta_sec = abs(delta_signed_sec)
            if best is None or delta_sec < best["delta_sec"]:
                best = {
                    "image": sample,
                    "delta_sec": delta_sec,
                    "delta_signed_sec": delta_signed_sec,
                }
        return best

    def _write_metadata(self, group_dir, metadata, rows):
        json_path = os.path.join(group_dir, "metadata.json")
        csv_path = os.path.join(group_dir, "metadata.csv")

        with open(json_path, "w") as f:
            json.dump(metadata, f, indent=2, sort_keys=True)

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "label",
                    "image_file",
                    "pcd_file",
                    "image_header_stamp",
                    "image_recv_stamp",
                    "cloud_header_stamp",
                    "cloud_recv_stamp",
                    "matched_cloud_stamp",
                    "trigger_stamp",
                    "sync_target_stamp",
                    "image_trigger_delta_ms",
                    "cloud_trigger_delta_ms",
                    "image_sync_target_delta_ms",
                    "cloud_sync_target_delta_ms",
                    "sync_delta_ms",
                    "point_count",
                    "fallback_to_center_cloud",
                    "pcd_data",
                    "pcd_fields",
                    "skipped_fields",
                ]
            )
            for row in rows:
                writer.writerow(
                    [
                        row["label"],
                        row["image_file"],
                        row["pcd_file"],
                        "{:.9f}".format(row["image_header_stamp"]),
                        "{:.9f}".format(row["image_recv_stamp"]),
                        "{:.9f}".format(row["cloud_header_stamp"]),
                        "{:.9f}".format(row["cloud_recv_stamp"]),
                        "{:.9f}".format(row["matched_cloud_stamp"]),
                        self._format_optional_float(row.get("trigger_stamp"), 9),
                        self._format_optional_float(row.get("sync_target_stamp"), 9),
                        self._format_optional_float(row.get("image_trigger_delta_ms"), 3),
                        self._format_optional_float(row.get("cloud_trigger_delta_ms"), 3),
                        self._format_optional_float(row.get("image_sync_target_delta_ms"), 3),
                        self._format_optional_float(row.get("cloud_sync_target_delta_ms"), 3),
                        "{:.3f}".format(row["sync_delta_ms"]),
                        row["point_count"],
                        row["fallback_to_center_cloud"],
                        row["pcd_data"],
                        " ".join(row["pcd_fields"]),
                        " ".join(row["skipped_fields"]),
                    ]
                )

    def _write_pcd(self, path, cloud):
        fields = []
        skipped_fields = []
        for field in cloud.fields:
            pcd_spec = POINT_FIELD_TO_PCD.get(field.datatype)
            if pcd_spec is None:
                skipped_fields.append(field.name)
                continue
            fields.append(
                {
                    "name": field.name,
                    "count": max(1, int(field.count)),
                    "size": pcd_spec[0],
                    "type": pcd_spec[1],
                    "offset": int(field.offset),
                }
            )

        if not fields:
            raise RuntimeError("PointCloud2 has no numeric fields that can be written to PCD")

        fields.sort(key=lambda item: item["offset"])
        field_names = [field["name"] for field in fields]
        point_count = int(cloud.width) * int(cloud.height)
        if point_count <= 0:
            raise RuntimeError("PointCloud2 contains no points to write")

        if self._can_write_pcd_binary_fast(cloud, fields, skipped_fields):
            self._write_pcd_binary_fast(path, cloud, fields, point_count)
            return {
                "point_count": point_count,
                "fields": field_names,
                "skipped_fields": skipped_fields,
                "data": "binary",
            }

        return self._write_pcd_ascii(path, cloud, fields, field_names, skipped_fields)

    def _can_write_pcd_binary_fast(self, cloud, fields, skipped_fields):
        if skipped_fields or cloud.is_bigendian:
            return False

        expected_offset = 0
        for field in fields:
            if field["offset"] != expected_offset:
                return False
            expected_offset += field["size"] * field["count"]

        return expected_offset == int(cloud.point_step)

    def _pcd_header_text(self, fields, width, height, data_type):
        point_count = int(width) * int(height)
        return (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS {}\n"
            "SIZE {}\n"
            "TYPE {}\n"
            "COUNT {}\n"
            "WIDTH {}\n"
            "HEIGHT {}\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS {}\n"
            "DATA {}\n"
        ).format(
            " ".join(field["name"] for field in fields),
            " ".join(str(field["size"]) for field in fields),
            " ".join(field["type"] for field in fields),
            " ".join(str(field["count"]) for field in fields),
            int(width),
            int(height),
            point_count,
            data_type,
        )

    def _write_pcd_binary_fast(self, path, cloud, fields, point_count):
        expected_bytes = point_count * int(cloud.point_step)
        data = cloud.data if isinstance(cloud.data, (bytes, bytearray)) else bytes(cloud.data)
        with open(path, "wb") as f:
            f.write(self._pcd_header_text(fields, cloud.width, cloud.height, "binary").encode("ascii"))
            f.write(data[:expected_bytes])

    def _write_pcd_ascii(self, path, cloud, fields, field_names, skipped_fields):
        points = []
        for point in point_cloud2.read_points(cloud, field_names=field_names, skip_nans=False):
            flat = []
            for value in point:
                if isinstance(value, float) and not math.isfinite(value):
                    flat.append("nan")
                else:
                    flat.append(value)
            points.append(flat)

        if not points:
            raise RuntimeError("PointCloud2 contains no points to write")

        with open(path, "w") as f:
            f.write(self._pcd_header_text(fields, len(points), 1, "ascii"))
            for point in points:
                f.write("{}\n".format(" ".join(self._format_pcd_value(value) for value in point)))

        return {
            "point_count": len(points),
            "fields": field_names,
            "skipped_fields": skipped_fields,
            "data": "ascii",
        }

    def _format_pcd_value(self, value):
        if value == "nan":
            return "nan"
        if isinstance(value, float):
            return "{:.9g}".format(value)
        return str(value)

    def _show_preview(self, image_sample, latest_cloud, pending, saving):
        now = rospy.Time.now().to_sec()
        if self.display_fps > 0.0 and now - self.last_preview_time < 1.0 / self.display_fps:
            return
        self.last_preview_time = now

        status = "press s: 3 frames, d: lidar-sync single, q: quit"
        color = (30, 180, 30)
        if latest_cloud is None:
            status = "waiting for point cloud"
            color = (20, 20, 220)
        elif saving:
            status = "saving in background | s/d disabled"
            color = (0, 180, 220)
        elif pending == "single_frame":
            status = "lidar-sync requested; waiting for strict pair"
            color = (0, 180, 220)
        elif pending is not None:
            status = "save requested; waiting for next image"
            color = (0, 180, 220)
        else:
            delta_ms = abs((image_sample["header_stamp"] - latest_cloud["recv_stamp"]).to_sec()) * 1000.0
            image_age_ms = max(0.0, (image_sample["recv_stamp"] - image_sample["header_stamp"]).to_sec() * 1000.0)
            status = "age {:.0f} ms | image/cloud {:.1f} ms | s:3 d:lidar q:quit".format(image_age_ms, delta_ms)

        if self.show_image_preview:
            image = image_sample["image"].copy()
            if 0.0 < self.preview_scale < 1.0:
                image = cv2.resize(
                    image,
                    None,
                    fx=self.preview_scale,
                    fy=self.preview_scale,
                    interpolation=cv2.INTER_AREA,
                )
            self._draw_status(image, status, color)
            cv2.imshow("hikvision image", image)

        if self.show_lidar_preview:
            cloud_view = self._render_cloud_view(latest_cloud["cloud"]) if latest_cloud else np.zeros((700, 700, 3), dtype=np.uint8)
            self._draw_status(
                cloud_view,
                "cloud recv stamp {:.6f}".format(latest_cloud["recv_stamp"].to_sec()) if latest_cloud else "waiting for point cloud",
                color,
            )
            cv2.imshow("hikvision lidar top view", cloud_view)

    def _draw_status(self, image, text, color):
        cv2.rectangle(image, (0, 0), (image.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(image, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)

    def _render_cloud_view(self, cloud):
        canvas_size = 700
        canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        center = canvas_size // 2
        scale = (canvas_size * 0.46) / max(self.display_range_m, 0.1)

        fields = {field.name.lower(): field.name for field in cloud.fields}
        x_name = fields.get("x")
        y_name = fields.get("y")
        if not x_name or not y_name:
            self._draw_status(canvas, "cloud has no x/y fields", (20, 20, 220))
            return canvas

        intensity_name = fields.get("intensity") or fields.get("reflectivity")
        field_names = (x_name, y_name, intensity_name) if intensity_name else (x_name, y_name)
        points_iter = point_cloud2.read_points(cloud, field_names=field_names, skip_nans=True)

        points = []
        for i, point in enumerate(points_iter):
            if i >= self.max_display_points:
                break
            x = float(point[0])
            y = float(point[1])
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            intensity = float(point[2]) if intensity_name else 80.0
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

    def _stamp_for_name(self, stamp):
        return "{:.6f}".format(stamp.to_sec()).replace(".", "_")

    def _stamp_float(self, stamp):
        return float(stamp.to_sec())

    def _format_optional_float(self, value, precision):
        if value is None:
            return ""
        return "{:.{}f}".format(float(value), precision)

    def _dedupe_group_dir(self, group_dir):
        for index in range(1, 1000):
            candidate = "{}_{:03d}".format(group_dir, index)
            if not os.path.exists(candidate):
                return candidate
        raise RuntimeError("failed to allocate unique group directory for {}".format(group_dir))

    def spin(self):
        rospy.spin()

    def close(self):
        cv2.destroyAllWindows()


def main():
    rospy.init_node("hikvision_collector_node")
    node = HikvisionCollectorNode()
    rospy.on_shutdown(node.close)
    node.spin()


if __name__ == "__main__":
    main()
