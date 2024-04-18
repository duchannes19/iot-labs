# Save this as patrolling_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Patrolling

class PatrollingClient(Node):

    def __init__(self):
        super().__init__('patrolling_client')
        self._action_client = ActionClient(self, Patrolling, 'patrolling')
        self._send_goal()

    def _send_goal(self):
        goal_msg = Patrolling.Goal()
        # Define your sequence of poses here, e.g., [(x1, y1), (x2, y2), ...]
        goal_msg.goal = [5.0, 5.0]
        self._action_client.wait_for_server()
        self.get_logger().info('Sending patrolling goal...')
        self._action_client.send_goal(goal_msg)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Feedback received: {}'.format(feedback_msg.feedback[0]))

def main(args=None):
    rclpy.init(args=args)
    patrolling_client = PatrollingClient()
    rclpy.spin(patrolling_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
