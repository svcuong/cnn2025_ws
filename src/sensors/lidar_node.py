#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

class LidarNode:
    # ============================================================================
    def __init__(self):
        rospy.init_node("lidar_node")

        # /rslidar_points được publish bởi node rslidar_sdk_node (tên là rslidar_driver), node này lấy dữ liệu trực tiếp từ LiDAR Helios qua UDP và convert thành "PointCloud2"

        # Load params - Đọc tham số từ launch / parameter server
        self.input_topic  = rospy.get_param("~input_topic", "/rslidar_points")
        self.output_topic = rospy.get_param("~output_topic", "/sensors/lidar/points")
        self.frame_id     = rospy.get_param("~frame_id", "lidar_link")

        # ============================================ PUBLISHER ============================================
        # --------------- Dữ liệu LiDAR đã chuẩn hóa frame_id ---------------
        self.pub_points = rospy.Publisher(
            self.output_topic, # Tên topic publish
            PointCloud2,  # Kiểu message ROS
            queue_size=5)  # Buffer nội bộ (ROS outgoing queue)
        

        # ---------------------- Gửi "OK" hoặc "ERROR" ----------------------
        self.pub_status = rospy.Publisher("/sensors/lidar/status", String, queue_size=1)


        # ====================== Subscriber - Node lắng nghe dữ liệu từ /rslidar_points =====================
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback, queue_size=5)

        # =============================================== LOG ===============================================
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
        msg.header.stamp = rospy.Time.now() 
        # normalize frame
        msg.header.frame_id = self.frame_id
        self.pub_points.publish(msg)  # Gửi dữ liệu vào ROS topic /sensors/lidar/points


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


# ====================================================================================================
# /rslidar_points  (PointCloud2)
#         │
#         ▼
#   self.sub  (Subscriber)
#         │
#         ▼
#    callback(msg)
#         │
#   đổi frame_id
#         │
#         ▼
#  self.pub_points.publish(msg)
#         │
#         ▼
# /sensors/lidar/points  (PointCloud2)


# [ LiDAR Helios (thiết bị thật) ]
#             ↓  UDP (6699 / 7788)
# [ rslidar_sdk_node  (node C++) ]
#             ↓  publish
#       /rslidar_points   (PointCloud2)
#             ↓  subscribe
# [ lidar_node.py  (node Python của anh) ]
#             ↓  publish
#  /sensors/lidar/points


# LOG ERROR:
# RegistrationIcp cannot do registration with a null guess (msg.header.stamp = rospy.Time.now() )
