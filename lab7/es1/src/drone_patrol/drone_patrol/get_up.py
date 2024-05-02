import rclpy
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Global Variables
MIN_ALTITUDE = 5
# Angle in quaternion
ROTATE_SECONDS = 5

# I have a bridge for the velocity and one for the odometry on a drone in gazebo fortress

# 1. Make the drone fly up (z) of a requested altitude
# #parameters=[{'direction': 0, 'altitude': randint(1, 10)}]

# Create a publisher for the velocity
# Create a subscriber for the odometry

# Make the drone rotate around the z axis of a requested angle




class MyDrone(Node):

    def __init__(self):
        super().__init__('my_drone')
        self.publisher_vel = self.create_publisher(Twist, 'X3/cmd_vel', 10)
        self.subscription_odom = self.create_subscription(Odometry, 'X3/odometry', self.odom_callback, 10)
        self.is_ready = False
        self.to_rotate = True


    def odom_callback(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.pose.pose.position.z)
        if msg.pose.pose.position.z < 5:
            self.get_logger().info('I need to fly up')
            self.fly_up(1.0)
        else:
            if(self.is_ready == True):
                self.fly_up(0.0)

            # Rotate the drone for 5 seconds
            if(self.to_rotate == True and self.is_ready == False):
                # Get the quaternion from the odometry
                x = msg.pose.pose.orientation.x
                y = msg.pose.pose.orientation.y
                z = msg.pose.pose.orientation.z
                w = msg.pose.pose.orientation.w

                yaw = euler_from_quaternion(x, y, z, w)
                self.get_logger().info('Yaw: [%s]' % str(yaw))
                self.rotate(1.0, ROTATE_SECONDS)
                return 0
    
    def fly_up(self, altitude):
        msg = Twist()
        msg.linear.z = altitude
        self.publisher_vel.publish(msg)
        self.get_logger().info('Publishing: [%f]' % msg.linear.z)
        if(altitude == 0.0):
            self.is_ready = True
            self.get_logger().info('I am already up')

    def rotate(self, angle, seconds):
        msg = Twist()
        msg.angular.z = angle
        self.publisher_vel.publish(msg)

        # Get the current time
        start_time = self.get_clock().now().to_msg()

        # Loop for the specified number of seconds
        while (self.get_clock().now().to_msg().sec - start_time.sec) < seconds:
            pass
        
        # Stop the drone
        msg.angular.z = 0.0
        self.publisher_vel.publish(msg)
        self.get_logger().info('Drone stopped')
        self.to_rotate = False
        
        

def euler_from_quaternion(x, y, z, w):
    
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +1.0 - 2.0 * (y * y + z * z)
    t4 = +2.0 * (w * z + x * y)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)

    publisher = MyDrone()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
