import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Pose as PoseMsg
from action_tutorials_interfaces.action import Patrolling
import math
from rclpy.executors import MultiThreadedExecutor

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
        #feedback_msg = Patrolling.Feedback()
        
        self.move_to_pose(goal)
        #feedback_pose = PoseMsg()
        #feedback_pose.position.x = self._current_pose.x
        #feedback_pose.position.y = self._current_pose.y
        #feedback_msg.feedback.append(feedback_pose)
        #goal_handle.publish_feedback(feedback_msg)
        while not self.reached_goal(goal):
            rclpy.spin_once(self)
        result = Patrolling.Result()
        result.result = goal
        goal_handle.succeed(result)

    def pose_callback(self, msg):
        self._current_pose = msg

    def move_to_pose(self, pose):
        #Log pose
        self.get_logger().info('Moving to pose: {}'.format(pose))
        angle = math.atan2(pose[1] - self._current_pose.y, pose[0] - self._current_pose.x)
        distance = math.sqrt((pose[0] - self._current_pose.x)**2 + (pose[1] - self._current_pose.y)**2)
        self.rotate(angle)
        self.move(distance)

    def rotate(self, angle):
        angular_speed = 0.5
        linear_speed = 0.0
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._twist_publisher.publish(twist)
        time = abs(angle / angular_speed)
        self.get_logger().info('Rotating for {} seconds'.format(time))
        self.create_timer(time, self.stop_moving)
    
    def stop_moving(self):
        twist = Twist()
        self._twist_publisher.publish(twist)
        
    def move(self, distance):
        angular_speed = 0.0
        linear_speed = 0.5
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
