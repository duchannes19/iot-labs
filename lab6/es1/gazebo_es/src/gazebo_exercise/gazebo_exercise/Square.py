import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MyRobot(Node):

    def __init__(self):
        super().__init__('gazebo_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.velocity = Twist()
        self.direction = 0

    def publish_velocity(self):
        if self.direction == 0:
            self.velocity.linear.x = 2.0  
            self.velocity.angular.z = 0.0
            self.direction += 1
        elif self.direction == 1:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 1.56
            self.direction = 0

        self.publisher.publish(self.velocity)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MyRobot()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
