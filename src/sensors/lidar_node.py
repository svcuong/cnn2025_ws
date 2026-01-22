#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

class LidarNode:
    # ============================================================================
    def __init__(self):
        rospy.init_node("lidar_node")

        # Load params - Đọc tham số từ launch / parameter server
        self.input_topic  = rospy.get_param("~input_topic", "/rslidar_points")
        self.output_topic = rospy.get_param("~output_topic", "/sensors/lidar/points")
        self.frame_id     = rospy.get_param("~frame_id", "lidar_link")

        # Publishers
        # Dữ liệu LiDAR đã chuẩn hóa frame_id
        self.pub_points = rospy.Publisher(self.output_topic, PointCloud2, queue_size=5)
        # Gửi "OK" hoặc "ERROR"
        self.pub_status = rospy.Publisher("/sensors/lidar/status", String, queue_size=1)

        # Subscriber - Node lắng nghe dữ liệu từ /rslidar_points 
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback, queue_size=5)

        self.last_msg_time = time.time()

        rospy.Timer(rospy.Duration(1.0), self.watchdog)

        rospy.loginfo("LiDAR adapter started")
        rospy.loginfo("Input : %s", self.input_topic)
        rospy.loginfo("Output: %s", self.output_topic)

    # ============================================================================
    # Xử lý dữ liệu LiDAR
    # Nhận PointCloud2 từ LiDAR
    # Đổi frame_id về lidar_link
    # Publish lại sang topic mới
    def callback(self, msg):

        
        self.last_msg_time = time.time()
        # normalize frame
        msg.header.frame_id = self.frame_id
        self.pub_points.publish(msg)
    # ============================================================================
    # Nếu hơn 1 giây không có dữ liệu LiDAR → báo "ERROR". Nếu LiDAR vẫn gửi đều → báo "OK"
    def watchdog(self, event):
        if time.time() - self.last_msg_time > 1.0:
            self.pub_status.publish("ERROR")
            rospy.logwarn("LiDAR timeout")
        else:
            self.pub_status.publish("OK")
    
    # ============================================================================
if __name__ == "__main__":
    try:
        LidarNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
