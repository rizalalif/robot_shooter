import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DroidCamPublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture("http://192.168.18.27:4747/video")  # Ganti dengan IP HP kamu
        # img_ref = cv2.imread('/root/ros2_ws/src/robot_shooter/robot_shooter/img/sengkuni1.jpg')
        # self.img_ref = "img/sengkuni1.jpg"
        if not self.cap.isOpened():
            self.get_logger().error("Failed open cam.")
        
        # self.get_logger().info("sending.....")
        # msg = self.bridge.cv2_to_imgmsg(img_ref, encoding='bgr8')
        # self.publisher_.publish(msg)
        # self.get_logger().info("success")
        

        # if not self.cap.isOpened():
        #     self.get_logger().error("Failed to open DroidCam stream.")
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        # img = self.cap
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Failed to read frame from DroidCam.")

def main(args=None):
    rclpy.init(args=args)
    node = DroidCamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
