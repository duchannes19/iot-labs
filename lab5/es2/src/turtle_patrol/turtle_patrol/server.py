import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Patrolling
import time
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist


class PatrollingActionServer(Node):
    
    def __init__(self):
        super().__init__('patrolling_action_server')
        self._action_server = ActionServer(
            self,
            Patrolling,
            'patrolling',
            self.execute_callback
        )
        self._turtlesim_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Patrolling.Feedback()
        feedback_msg.current_position = [0, 0]
        
        for i in range(1, goal_handle.request.points):
            feedback_msg.current_position = goal_handle.request.points[i]
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_position))
            goal_handle.publish_feedback(feedback_msg)
            
            # Publish twist message to control the turtle's movement
            twist_msg = Twist()
            twist_msg.linear.x = 1.0  # Set linear velocity
            twist_msg.angular.z = 0.5  # Set angular velocity
            self._turtlesim_publisher.publish(twist_msg)
            
            time.sleep(1)
        
        # Stop the turtle's movement
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self._turtlesim_publisher.publish(twist_msg)
        
        goal_handle.succeed()
        
        result = Patrolling.Result()
        result.final_position = feedback_msg.current_position
        return result


def main():
    rclpy.init()

    executor = MultiThreadedExecutor()
    patrolling_action_server = PatrollingActionServer()
    executor.add_node(patrolling_action_server)

    executor.spin()