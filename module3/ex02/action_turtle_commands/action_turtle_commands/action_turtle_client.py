# action_turtle_commands/action_turtle_client.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from actions.action import MessageTurtleCommands

class TurtleActionClient(Node):
    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'MessageTurtleCommands')

    def send_goal(self, command, s=0, angle=0):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Command result: {result.result}')
        rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.odom} meters traveled')


def main(args=None):
    rclpy.init(args=args)
    action_client = TurtleActionClient()

    # Программа движений: 2 метра вперед, поворот на 90 градусов направо, 1 метр вперед
    action_client.send_goal('forward', s=2)
    #rclpy.spin_once(action_client)

    # После первого действия можно запускать следующее

    action_client.send_goal('turn_right', angle=90)
    #rclpy.spin_once(action_client)

    action_client.send_goal('forward', s=1)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

