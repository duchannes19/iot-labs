import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from turtlesim.srv import SetPen

# With rotation and change color of the line

class MyTurtle(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')  
        self.set_pen_request = SetPen.Request()
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
            self.direction += 1
        elif self.direction == 2:
            self.velocity.linear.x = 2.0
            self.velocity.angular.z = 0.0
            self.direction += 1
        elif self.direction == 3:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z =  1.56
            self.direction += 1
        elif self.direction == 4:
            self.velocity.linear.x = 2.0 
            self.velocity.angular.z = 0.0
            self.direction += 1
        elif self.direction == 5:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z =  1.56
            self.direction += 1
        elif self.direction == 6:
            self.velocity.linear.x = 2.0
            self.velocity.angular.z = 0.0
            self.direction += 1
        elif self.direction == 7:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z =  1.56
            self.direction = 0

        # Call the set_pen service to set the color
        # The color is random between 0 to 255
        color = ColorRGBA()
        color.r = random.uniform(0, 255)
        color.g = random.uniform(0, 255)
        color.b = random.uniform(0, 255)

        self.set_pen_request.r = int(color.r)
        self.set_pen_request.g = int(color.g)
        self.set_pen_request.b = int(color.b)
        self.set_pen_request.width = 3
        self.set_pen_request.off = False
        self.set_pen_client.call_async(self.set_pen_request)

        self.publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MyTurtle()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
