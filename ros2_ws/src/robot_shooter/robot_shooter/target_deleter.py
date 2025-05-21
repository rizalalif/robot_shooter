import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity

class TargetDeleter(Node):
    def __init__(self):
        super().__init__('target_deleter')
        self.client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity service...')
        self.models = ['patok_depan', 'patok_belakang', 'patok_kiri', 'patok_kanan']
        self.delete_models()

    def delete_models(self):
        for model in self.models:
            request = DeleteEntity.Request()
            request.name = model
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Successfully deleted: {model}')
            else:
                self.get_logger().warn(f'Failed to delete: {model}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetDeleter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
