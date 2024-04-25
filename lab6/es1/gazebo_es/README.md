# Solution

```
# Source the ROS setup.bash file
source /opt/ros/humble/setup.bash

# Build the colcon workspace
colcon build

# Source the local_setup.bash file
source ./install/local_setup.bash

# Set environment variables for rendering in software mode
export LIBGL_ALWAYS_INDIRECT=0
export LIBGL_ALWAYS_SOFTWARE=1

# Run the ros_gz_bridge parameter_bridge in the background
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist &

# Launch the launch_gazebo.py script
ros2 launch launch_gazebo.py
```