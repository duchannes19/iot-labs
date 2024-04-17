import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Patrolling

class PatrollingActionClient(Node):
    
    def __init__(self):
        super().__init__('patrolling_action_client')
        self._action_client = ActionClient(self, Patrolling, 'patrolling')
    
    def send_goal(self, points):
        goal_msg = Patrolling.Goal()
        goal_msg.points = points
        
        self._action_client.wait_for_server()
        self._action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.current_position))
    
    def get_result(self):
        self._action_client.wait_for_result()
        return self._action_client.get_result()


def main():
    rclpy.init()


    patrolling_action_client = PatrollingActionClient()
    points = [1, 2]
    patrolling_action_client.send_goal(points)

    result = patrolling_action_client.get_result()
    if result:
        patrolling_action_client.get_logger().info('Final position: {0}'.format(result.final_position))
    else:
        patrolling_action_client.get_logger().info('Action failed')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
