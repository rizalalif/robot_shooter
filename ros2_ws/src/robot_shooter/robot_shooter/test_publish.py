import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker")
        self.publisher_ =  self.create_publisher(String,"whatsapp",10)
        self.timer_ = self.create_timer(1,self.sending)
        self.get_logger().info("Sending...")


    def sending(self):
        msg = String()
        msg.data = "Hallo"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()