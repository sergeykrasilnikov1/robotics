import sys
import rclpy
from rclpy.node import Node
from full_name_pkg.srv import FullNameSumService

class SummFullNameClient(Node):
    def __init__(self):
        super().__init__('summ_full_name_client')
        self.client = self.create_client(FullNameSumService, 'SummFullName')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = FullNameSumService.Request()
        self.request.last_name = sys.argv[1]
        self.request.first_name = sys.argv[2]
        self.request.name = sys.argv[3]

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Full name: %s' % self.future.result().full_name)
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run service_full_name client_name <last_name> <first_name> <middle_name>")
        return
    node = SummFullNameClient()
    node.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
