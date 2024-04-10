# Exercise 4 - Spin the Turtle

## Setup

1. Create two folders (ex: es1, es2)

2. Inside those folders create a src folder.

3. In es1\src execute this:

```
 ros2 pkg create --build-type ament_cmake tutorial_interfaces 
```

4. In es2\src execute this:

```
 ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy 
```

## Create the interface for the Spin service

1. In es2\src\tutorial_interfaces add these:

```
# Below the find dependencies
find_package(rosidl_default_generators REQUIRED)

...

# Before ament_package
rosidl_generate_interfaces(${PROJECT_NAME}
    # Exercise to spin the turtle
    "srv/Spin.srv"
)
```

2. In the tutorial_interfaces folder create a srv folder.

3. In the srv folder create a file called Spin.srv with inside:

```
string direction
---
string message
```

4. Compile with colcon build in the first src folder (.\es1).

## Create the client and the server services that uses the Spin interface

1. In es2\src\python_parameters\python_parameters create two files: Spin_Client.py and Spin_Server.py.

2. In Spin_Client.py:

```
import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import Spin

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spin_client')

    client = node.create_client(Spin, 'spin')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    while rclpy.ok():
        request = Spin.Request()
        request.direction = input('Enter direction (left/right/quit): ')

        if request.direction.lower() == 'quit':
            rclpy.shutdown()
            node.destroy_node()
            return 0

        future = client.call_async(request)

        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()
            node.get_logger().info('Response: %s' % response.message)
        else:
            node.get_logger().info('Service call failed')

if __name__ == '__main__':
    main()
```

3. In Spin_Server.py:

```
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
```

4. Add the services to setup.py:

```
entry_points={
    'console_scripts': [
        'spin_server = python_parameters.Spin_Server:main',
        'spin_client = python_parameters.Spin_Client:main',
    ],
},
```

5. Compile in main folder with colcon build.

6. (Optional) Use launch.py to start the turtle and/or the turtle.

## Launch the services

1. In two terminal you have to source both the builds in es1 and in es2 and ros to find the dependencies to correctly start the services.

```
source /opt/ros/humble/setup.bash
# In es1 folder
source ./install/local_setup.bash
# In es2 folder
source ./install/local_setup.bash
```

2. In a third terminal start the turtle:

```
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

3. Start the server:

```
ros2 run python_parameters spin_server
```


4. Start the client:

```
ros2 run python_parameters spin_client
```