# Ex 1

You are requested to write a ROS publisher which interfaces with TurtleSim!

Now that you are able to write a more complicated behavior than a simple movement, you should be able to program the turtle to do something more fancy.

You are here requested to move the turtle in a square path.

## Solution

1. In a new folder create the src directory:

```
mkdir es1
cd es1
mkdir src
```

2. Inside the src folder execute this command:

```
ros2 pkg create --build-type ament_python py_pubsub
```

3. Navigate two times inside the py_pubsub folder (where the init file is), and execute:

```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

3. (Optional) Rename the example publisher as you like.

4. Change the publisher member class to be like this (This is without rotation):

```
from geometry_msgs.msg import Twist

class MyTurtle(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.velocity = Twist()
        self.direction = 0

    def publish_velocity(self):
        if self.direction == 0:
            self.velocity.linear.x = 2.0  # move left
            self.velocity.linear.y = 0.0
            self.direction += 1
        elif self.direction == 1:
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = 2.0  # move up
            self.direction += 1
        elif self.direction == 2:
            self.velocity.linear.x = -2.0  # move right
            self.velocity.linear.y = 0.0
            self.direction += 1
        elif self.direction == 3:
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = -2.0 # move down
            self.direction = 0

        self.publisher.publish(self.velocity)
```

Also when it is called inside the main function:

```
    minimal_publisher = MyTurtle()
```

5. Add the publisher to the setup.py:

```
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.MySquare:main',
        ],
    },
```

I think 'talker' can be any name.

6. Source ros before compiling:

```
source /opt/ros/humble/setup.bash
```

7. Build the src in the above directory:

```
# You should be in you es folder not the src
colcon build
```

8. Source your build:

```
source ./install/local_setup.bash
```

9. In a different terminal start turtlesim:

```
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

10. In the previous terminal run your package:

```
ros2 run py_pubsub talker
```

## To add Color see MySquare3