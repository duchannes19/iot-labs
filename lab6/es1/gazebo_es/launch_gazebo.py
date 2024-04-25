from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from random import randint

WORLD_FILE = "hello_world.sdf"

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
      # Launch the publisher node
      Node(
        package='gazebo_exercise',
        executable='gz_es1',
        name='gz_es1',
        output='screen',
        emulate_tty=True
      )
    ]
  )