# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time", default_value="false", choices=["true", "false"], description="Use sim time"
    ),
]


def generate_launch_description():
    pkg_jackal_navigation = FindPackageShare("jackal_navigation")

    slam_config = PathJoinSubstitution([pkg_jackal_navigation, "config", "slam.yaml"])

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam)
    return ld

# ### cartographer
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import ThisLaunchFileDir



# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
#     cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
#                                                   turtlebot3_cartographer_prefix, 'config'))
#     configuration_basename = LaunchConfiguration('configuration_basename',
#                                                  default='jackal_2d.lua')
#     resolution = LaunchConfiguration('resolution', default='0.05')
#     publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

#     rviz_config_dir = os.path.join(get_package_share_directory('jackal_gazebo'),#turtlebot3_cartographer
#                                 'rviz', '/home/bj/ros2_ws/src/jackal_simulator/jackal_gazebo/rviz/jackal.rviz')#tb3_cartographer.rviz
    
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'cartographer_config_dir',
#             default_value=cartographer_config_dir,
#             description='Full path to config file to load'),
#         DeclareLaunchArgument(
#             'configuration_basename',
#             default_value=configuration_basename,
#             description='Name of lua file for cartographer'),
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='true',
#             description='Use simulation (Gazebo) clock if true'),

#         Node(
#             package='cartographer_ros',
#             executable='cartographer_node',
#             name='cartographer_node',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time}],
#             arguments=['-configuration_directory', cartographer_config_dir,
#                        '-configuration_basename', configuration_basename]),

#         DeclareLaunchArgument(
#             'resolution',
#             default_value=resolution,
#             description='Resolution of a grid cell in the published occupancy grid'),

#         DeclareLaunchArgument(
#             'publish_period_sec',
#             default_value=publish_period_sec,
#             description='OccupancyGrid publishing period'),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
#             launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
#                               'publish_period_sec': publish_period_sec}.items(),
#         ),


#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_dir],
#             parameters=[{'use_sim_time': use_sim_time}],
#             output='screen'),
#     ])
