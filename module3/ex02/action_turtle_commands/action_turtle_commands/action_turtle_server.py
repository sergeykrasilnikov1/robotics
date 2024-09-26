import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_msgs.msg import GoalStatus
from actions.action import MessageTurtleCommands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleActionServer(Node):
    def __init__(self):
        super().__init__('action_turtle_server')

        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'MessageTurtleCommands',
            self.execute_callback
        )

        self._pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self._cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.current_pose = Pose()

    def pose_callback(self, pose):
        self.current_pose = pose

    def execute_callback(self, goal_handle):
        feedback_msg = MessageTurtleCommands.Feedback()

        if self.current_pose is None:
            self.get_logger().error('Current pose is not available yet.')
            goal_handle.abort()
            return MessageTurtleCommands.Result(result=False)

        twist = Twist()
        if goal_handle.request.command == "forward":
            self.get_logger().info(f'Moving forward {goal_handle.request.s} meters.')
            start_x = self.current_pose.x
            start_y = self.current_pose.y
            distance_travelled = 0.0

            twist.linear.x = 1.0  # Скорость движения вперёд
            while distance_travelled < goal_handle.request.s:
                self._cmd_vel_publisher.publish(twist)
                rclpy.spin_once(self)
                distance_travelled = math.sqrt(
                    (self.current_pose.x - start_x) ** 2 +
                    (self.current_pose.y - start_y) ** 2
                )
                feedback_msg.odom = int(distance_travelled)
                goal_handle.publish_feedback(feedback_msg)

            twist.linear.x = 0.0
            self._cmd_vel_publisher.publish(twist)

        elif goal_handle.request.command in ["turn_left", "turn_right"]:
            angle_to_turn = goal_handle.request.angle
            self.get_logger().info(f'Turning {angle_to_turn} degrees.')

            twist.angular.z = 1.0 if goal_handle.request.command == "turn_left" else -1.0

            initial_angle = self.current_pose.theta
            angle_turned = 0.0

            while angle_turned < math.radians(abs(angle_to_turn)):
                self._cmd_vel_publisher.publish(twist)
                rclpy.spin_once(self)
                angle_turned = abs(self.current_pose.theta - initial_angle)

            twist.angular.z = 0.0
            self._cmd_vel_publisher.publish(twist)

        goal_handle.succeed()
        return MessageTurtleCommands.Result(result=True)


def main(args=None):
    rclpy.init(args=args)
    action_server = TurtleActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

