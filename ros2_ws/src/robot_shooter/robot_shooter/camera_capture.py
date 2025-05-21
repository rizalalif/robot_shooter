import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import os

class CameraCapture(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Ganti dengan topic dari kamera gazebo kamu
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.saved = False  # Hanya simpan satu kali untuk demo

    def listener_callback(self, msg):
        if not self.saved:
            self.saved = True
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            filepath = os.path.join('/root/ros2_ws/src/robot_shooter/robot_shooter/img', filename)
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"Gambar disimpan ke: {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
