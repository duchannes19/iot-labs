import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point  # Import the Point message type
from exercise8_interfaces.action import Patrol

# Patrol is an array of points that the drone has to visit

class DronePatrol(Node):
    def __init__(self):
        # Initialize ROS node and action client
        super().__init__('drone_patrol')
        self.action_client = ActionClient(self, Patrol, 'patrol')
        self.patrol()

    def patrol(self):
        # Wait for the server to be available
        self.action_client.wait_for_server()
        
        # Create a new goal message with Point objects
        goal_msg = Patrol.Goal()
        goal_msg.target = [
            Point(x=0.0, y=0.0, z=1.0),
            Point(x=1.0, y=0.0, z=1.0),
            Point(x=1.0, y=1.0, z=1.0),
            Point(x=0.0, y=1.0, z=1.0),
            Point(x=0.0, y=0.0, z=1.0)
        ]
        
        # Send the goal and wait for the result
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
    

def main():
    rclpy.init()
    drone_patrol = DronePatrol()
    rclpy.spin(drone_patrol)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
