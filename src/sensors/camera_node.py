#!/usr/bin/env python3
# License: Apache 2.0
# Copyright(c) Intel Corporation
# Subscriber node for Intel RealSense (ROS1)

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class RealSenseSubscriberNode:
    def __init__(self):
        # ===============================
        # Init ROS node
        # ===============================
        rospy.init_node('camera_node', anonymous=False)

        # Namespace camera (mặc định: /camera)
        self.camera_ns = rospy.get_param("~camera_ns", "camera")

        # Topics theo realsense2_camera
        self.color_topic = f"/{self.camera_ns}/color/image_raw"
        self.depth_topic = f"/{self.camera_ns}/depth/image_rect_raw"
        self.color_info_topic = f"/{self.camera_ns}/color/camera_info"


        self.bridge = CvBridge()
        # ===============================
        # Subscribers
        # ===============================
        self.color_sub = rospy.Subscriber(
            self.color_topic,
            Image,
            self.color_callback,
            queue_size=1
        )

        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            Image,
            self.depth_callback,
            queue_size=1
        )

        self.color_info_sub = rospy.Subscriber(
            self.color_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        rospy.loginfo("RealSense Subscriber Node started")
        rospy.loginfo("Subscribing to:")
        rospy.loginfo(f"  Color : {self.color_topic}")
        rospy.loginfo(f"  Depth : {self.depth_topic}")

    # ===============================
    # Callbacks
    # ===============================
    def color_callback(self, msg: Image):
        try:
            # Color image (BGR8)
            color_img = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )

            # 👉 dùng color_img cho SLAM / xử lý CV ở đây
            # ví dụ: ORB, feature, publish sang node khác, v.v.

        except Exception as e:
            rospy.logerr(f"Color callback error: {e}")

    def depth_callback(self, msg: Image):
        try:
            # Depth image (16UC1, mm)
            depth_img = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )

            # 👉 dùng depth_img cho SLAM / depth processing

        except Exception as e:
            rospy.logerr(f"Depth callback error: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        # Log 1 lần thông số nội tại camera
        rospy.loginfo_once("Camera Info received")
        rospy.loginfo_once(f"Width: {msg.width}, Height: {msg.height}")
        rospy.loginfo_once(f"K: {msg.K}")

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RealSenseSubscriberNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
