import rclpy
from rclpy.node import Node
from full_name_pkg.srv import FullNameSumService

class FullNameService(Node):
    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(FullNameSumService, 'SummFullName', self.handle_full_name_request)

    def handle_full_name_request(self, request, response):
        response.full_name = f"{request.last_name} {request.first_name} {request.name}"
        self.get_logger().info(f"Sending response: {response.full_name}")
        return response

def main(args=None):
    rclpy.init(args=args)
    service = FullNameService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

