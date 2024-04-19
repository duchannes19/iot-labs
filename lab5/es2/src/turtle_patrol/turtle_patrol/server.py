import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Pose as PoseMsg
import math
from rclpy.executors import MultiThreadedExecutor
from turtle_patrol_actions.action import Patrolling

class PatrollingServer(Node):

    def __init__(self):
        super().__init__('patrolling_server')
        self._action_server = ActionServer(
            self,
            Patrolling,
            'patrolling',
            self.execute_callback)
        self._pose_subscriber = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self._current_pose = Pose()
        self._twist_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing patrolling action...')
        goal = goal_handle.request.goal
        feedback_msg = Patrolling.Feedback()
        
        self.move_to_pose(goal)
        feedback_pose = PoseMsg()
        feedback_pose.position.x = self._current_pose.x
        feedback_pose.position.y = self._current_pose.y
        feedback_msg.feedback.append(feedback_pose)
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Moving...')
        while not self.reached_goal(goal):
            self.move_to_pose(goal)
            
        # Send the result
        result = Patrolling.Result()
        print(type(result.result))
        result.result = True
        goal_handle.succeed(result)
        return result

    def pose_callback(self, msg):
        self._current_pose = msg

    def move_to_pose(self, pose):
        #Log pose
        #self.get_logger().info('Moving to pose: {}'.format(pose))
        angle = math.atan2(pose[1] - self._current_pose.y, pose[0] - self._current_pose.x)
        distance = math.sqrt((pose[0] - self._current_pose.x)**2 + (pose[1] - self._current_pose.y)**2)
        # Log angle and distance
        #self.get_logger().info('Angle: {}'.format(angle))
        #self.get_logger().info('Distance: {}'.format(distance))
        # First rotate and wait then stop rotating and move
        self.rotate(angle, distance)
        
    
    def stop_moving(self):
        twist = Twist()
        self._twist_publisher.publish(twist)

    def rotate(self, angle, distance):
        # Calculate the angle difference between the current pose and the goal pose
        angle_difference = angle - self._current_pose.theta

        # Normalize the angle difference to be between -pi and pi
        while angle_difference > math.pi:
            angle_difference -= 2 * math.pi
        while angle_difference < -math.pi:
            angle_difference += 2 * math.pi

        # Set the angular speed based on the angle difference
        angular_speed = 0.5 if angle_difference > 0 else -0.5

        # Rotate the turtle until it faces the goal
        while abs(angle_difference) > 0.1:
            twist = Twist()
            twist.angular.z = angular_speed
            self._twist_publisher.publish(twist)
            angle_difference = angle - self._current_pose.theta

        # Stop rotating
        self.stop_moving()
        self.move(distance)
        
    def move(self, distance):
        angular_speed = 0.0
        linear_speed = 1.0
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._twist_publisher.publish(twist)
        time = distance / linear_speed
        self.get_logger().info('Moving for {} seconds'.format(time))
        self.create_timer(time, self.stop_moving)
    
    def reached_goal(self, pose):
        distance = math.sqrt((pose[0] - self._current_pose.x)**2 + (pose[1] - self._current_pose.y)**2)
        return distance < 0.1
    
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    patrolling_server = PatrollingServer()
    executor.add_node(patrolling_server)
    executor.spin()

if __name__ == '__main__':
    main()
