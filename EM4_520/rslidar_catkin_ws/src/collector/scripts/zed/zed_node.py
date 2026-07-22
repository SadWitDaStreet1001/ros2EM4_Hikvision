#!/usr/bin/env python3

import threading

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ZedNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/zed/image_raw")
        self.left_image_topic = rospy.get_param("~left_image_topic", "/zed/left/image_raw")
        self.right_image_topic = rospy.get_param("~right_image_topic", "/zed/right/image_raw")
        self.frame_id = rospy.get_param("~frame_id", "zed_camera")
        self.left_frame_id = rospy.get_param("~left_frame_id", "zed_left_camera")
        self.right_frame_id = rospy.get_param("~right_frame_id", "zed_right_camera")
        self.camera_index = int(rospy.get_param("~camera_index", 0))
        self.video_device = rospy.get_param("~video_device", "")
        self.width = int(rospy.get_param("~width", 3840))
        self.height = int(rospy.get_param("~height", 1080))
        self.fps = float(rospy.get_param("~fps", 30.0))
        self.show_preview = bool(rospy.get_param("~show_preview", True))
        self.preview_scale = float(rospy.get_param("~preview_scale", 0.5))

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=5)
        self.left_pub = rospy.Publisher(self.left_image_topic, Image, queue_size=5)
        self.right_pub = rospy.Publisher(self.right_image_topic, Image, queue_size=5)
        self.stop_event = threading.Event()

        source = self.video_device if self.video_device else self.camera_index
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError("failed to open ZED camera source: {}".format(source))

        if self.width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo(
            "zed_node source=%s requested=%dx%d actual=%dx%d fps=%.1f",
            source,
            self.width,
            self.height,
            actual_w,
            actual_h,
            self.fps,
        )
        if actual_w % 2 != 0:
            rospy.logwarn("ZED frame width %d is odd; stereo split may be invalid", actual_w)

    def _split_stereo(self, frame):
        height, width = frame.shape[:2]
        if width < 2 or width % 2 != 0:
            raise RuntimeError("invalid stereo frame size: {}x{}".format(width, height))
        half = width // 2
        return frame[:, :half].copy(), frame[:, half:].copy()

    def _make_preview(self, left, right):
        preview = cv2.hconcat([left, right])
        cv2.rectangle(preview, (0, 0), (preview.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(
            preview,
            "zed preview | press q to quit",
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

    def spin(self):
        rate_hz = self.fps if self.fps > 0 else 30.0
        rate = rospy.Rate(rate_hz)
        rospy.loginfo(
            "zed_node publishing %s, %s, %s",
            self.image_topic,
            self.left_image_topic,
            self.right_image_topic,
        )

        while not rospy.is_shutdown() and not self.stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok:
                rospy.logwarn_throttle(2.0, "ZED frame grab failed")
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
            self.image_pub.publish(combined_msg)

            left_msg = self.bridge.cv2_to_imgmsg(left, encoding="bgr8")
            left_msg.header.stamp = stamp
            left_msg.header.frame_id = self.left_frame_id
            self.left_pub.publish(left_msg)

            right_msg = self.bridge.cv2_to_imgmsg(right, encoding="bgr8")
            right_msg.header.stamp = stamp
            right_msg.header.frame_id = self.right_frame_id
            self.right_pub.publish(right_msg)

            if self.show_preview:
                cv2.imshow("zed preview", self._make_preview(left, right))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    rospy.signal_shutdown("ZED preview closed")

            rate.sleep()

    def close(self):
        self.stop_event.set()
        if self.cap is not None:
            self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("zed_node")
    node = None
    try:
        node = ZedNode()
        rospy.on_shutdown(node.close)
        node.spin()
    except Exception as exc:
        rospy.logerr("%s", exc)
        if node is not None:
            node.close()
        raise


if __name__ == "__main__":
    main()
