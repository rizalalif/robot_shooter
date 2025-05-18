import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__("listener")
        self.subscription_ = self.create_subscription(String,"whatsapp",self.getMessage,10)



    def getMessage(self,msg):
        # msg = String()
        # msg.data = "Hallo"]
        data = msg.data
        self.get_logger().info(f"from talker : {data}")

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()