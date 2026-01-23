#!/usr/bin/env python3
# License: Apache 2.0
# Copyright(c) Intel Corporation
# Subscriber node for Intel RealSense (ROS1)

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

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
        rospy.loginfo(f"Subscribing to:")
        rospy.loginfo(f"  Color : {self.color_topic}")
        rospy.loginfo(f"  Depth : {self.depth_topic}")

    # ===============================
    # Callbacks
    # ===============================
    def color_callback(self, msg: Image):
        try:
            # RealSense color image: BGR8
            color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("RealSense Color", color_img)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Color callback error: {e}")

    def depth_callback(self, msg: Image):
        try:
            # Depth image: 16UC1 (millimeters)
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Normalize for visualization
            depth_vis = cv2.normalize(
                depth_img, None, 0, 255, cv2.NORM_MINMAX
            ).astype('uint8')

            cv2.imshow("RealSense Depth", depth_vis)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Depth callback error: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        # In ra thông số nội tại camera (chỉ log 1 lần)
        rospy.loginfo_once("Camera Info received")
        rospy.loginfo_once(f"Width: {msg.width}, Height: {msg.height}")
        rospy.loginfo_once(f"K: {msg.K}")

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = RealSenseSubscriberNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
