import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    # Rviz 실행
    sim_time = LaunchConfiguration('sim_time')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            
            arguments=['-d', os.path.join(get_package_share_directory('jackal_gazebo'), 
                                          'rviz', '/home/bj/ros2_ws/src/jackal_simulator/jackal_gazebo/rviz/jackal.rviz')]
        )
    ])