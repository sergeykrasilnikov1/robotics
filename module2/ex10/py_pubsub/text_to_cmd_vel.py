import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TextToCmdVel(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')

        self.subscription = self.create_subscription(String,'cmd_text',self.cmd_text_callback,10)
        self.subscription 

        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def cmd_text_callback(self, msg):

        twist = Twist()

        if msg.data == 'move_forward':
            twist.linear.x = 1.0  
            twist.angular.z = 0.0
        elif msg.data == 'move_backward':
            twist.linear.x = -1.0  
            twist.angular.z = 0.0
        elif msg.data == 'turn_left':
            twist.linear.x = 0.0
            twist.angular.z = 1.5 
        elif msg.data == 'turn_right':
            twist.linear.x = 0.0
            twist.angular.z = -1.5  
        else:
            self.get_logger().info(f"Неизвестная команда: {msg.data}")
            return

        self.publisher_.publish(twist)
        self.get_logger().info(f'Опубликована команда: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    text_to_cmd_vel = TextToCmdVel()

    rclpy.spin(text_to_cmd_vel)

    text_to_cmd_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

