from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from random import randint

WORLD_FILE = "drone_world.sdf"

def generate_launch_description():

  return LaunchDescription(
    [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
              FindPackageShare('ros_gz_sim'),
              'launch/'
              'gz_sim.launch.py',
          ])
        ]),
        launch_arguments={
          'gz_args' : "./"+WORLD_FILE
        }.items()
      ),
      Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          "/X3/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
        ],
        name='cmd_vel_bridge'
      ),
      Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          "/X3/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
        ],
        name='odometry_bridge'
      ),
      Node(
        package='exercise8_src',
        executable='drone_controller',
        name='drone_controller',
        namespace='X3'
      ),
      Node(
        package='exercise8_src',
        executable='drone_client',
        name='drone_client',
        namespace='X3'
      )
    ]
  )
