import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time

class Happy(Node):
    def __init__(self):
        super().__init__('happy')

        self.velocity_publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        
        self.pose_subscriber = self.create_subscription(Image, '/depth/image', self.update_pose, 10)
        self.scan = Image()

        self.timer = self.create_timer(0.5, self.move)

    def update_pose(self, data):
        self.scan = data

    def move(self):
        
        vel_msg = Twist()

        if len(self.scan.data) != 0:
            mid_index = int(self.scan.width * self.scan.height / 2 + self.scan.width / 2)
            depth_value = int(self.scan.data[mid_index])

            self.get_logger().info(f'Current depth value at center: {depth_value}')
            
            if not depth_value:
                vel_msg.linear.x = 0.3
            else:
                vel_msg.linear.x = 0.0

        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    time.sleep(2)

    node = Happy()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
