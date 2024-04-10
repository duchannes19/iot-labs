import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tutorial_interfaces.srv import Spin  # Import the custom service message

class SpinService(Node):
    def __init__(self):
        super().__init__('spin_server')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.service = self.create_service(Spin, 'spin', self.spin_callback)

    def spin_callback(self, request, response):
        twist = Twist()

        #Print the request
        self.get_logger().info('Incoming request: direction = %s' % request.direction.lower())

        if request.direction.lower() == 'left':
            twist.angular.z = 10.0
        elif request.direction.lower() == 'right':
            twist.angular.z = -10.0
        else:
            response.message = 'Invalid direction. Please specify "left" or "right".'
            return response

        self.publisher.publish(twist)
        response.message = 'Spin started in the ' + request.direction + ' direction.'
        return response

def main(args=None):
    rclpy.init(args=args)
    spin_service = SpinService()
    rclpy.spin(spin_service)
    spin_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()