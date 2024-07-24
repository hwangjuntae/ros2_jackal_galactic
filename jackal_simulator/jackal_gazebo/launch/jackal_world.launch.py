import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    world_file = PathJoinSubstitution(
        [FindPackageShare("jackal_gazebo"), "worlds", "HRI_lab.world"],
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("jackal_gazebo"), "launch", "gazebo.launch.py"],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={"world_path": world_file}.items(),
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            
            arguments=['-d', os.path.join(get_package_share_directory('jackal_gazebo'), 
                                          'rviz', '/home/teus/jackal_ws/src/jackal_simulator/jackal_gazebo/rviz/jackal.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_sim)
    ld.add_action(rviz)
    
    return ld
