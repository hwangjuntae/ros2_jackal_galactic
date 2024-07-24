from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import os
from os import environ, pathsep

# ARGUMENTS = [
#     DeclareLaunchArgument(
#         "world_path", default_value="5th_floor", description="The world path, by default is empty.world"
#     ),
# ]

def generate_launch_description():
    # use_sim_time = True

    # robot_description_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('jackal_description'), 'launch', 'description.launch.py']
    # )

    # gz_resource_path = SetEnvironmentVariable( #same
    # name="GAZEBO_MODEL_PATH",
    # value=[
    #     EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
    #     "/usr/share/gazebo-11/models/:",
    #     str(Path(get_package_share_directory("jackal_description")).parent.resolve()),
    # ])

    # Launch args #same
    # world_path = LaunchConfiguration('world_path')
    # prefix = LaunchConfiguration('prefix')
    declare_world_name = DeclareLaunchArgument(
        'world_name', default_value='khu_5th_floor',
        description='Specify world name, we\'ll convert to full path'
    )


    config_jackal_velocity_controller = PathJoinSubstitution( #same
        [FindPackageShare("jackal_control"), "config", "control2.yaml"]
    )

    # Get URDF via xacro # xacro 파일 수정 #same
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("jackal_description"), "urdf", "jackal_base.urdf.xacro"]
        ),
        " ",
        "is_sim:=true",
        " ",
        "gazebo_controllers:=",
        config_jackal_velocity_controller,
    ]

    launch_jackal_description = IncludeLaunchDescription( #same
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("jackal_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments=[("robot_description_command", robot_description_command)],
        # launch_arguments={'robot_description_command':robot_description_command, 'is_sim':'True',}.items()
    )

    #gazebo server, client in example_cafe2.launch.py

    # declare_model_name = DeclareLaunchArgument(
    #     'model_name', default_value='jackal_kinova',
    #     description='Gazebo model name'
    # )
    
    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_jackal",
        arguments=["-entity", "jackal", "-topic", "robot_description"],
        output="screen",
    )
    # Launch jackal_control/control.launch.py
    # control.launch 바꿨더니 gazebo 오류남
    launch_jackal_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("jackal_control"), "launch", "hunav_control.launch.py"]
            )
        ),
        # launch_arguments={'robot_description_command':robot_description_command, 'is_sim':'True',}.items()
        launch_arguments=[
            ("robot_description_command", robot_description_command),
            ("is_sim", "True")]
    )

    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py'])))
    
    # IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(robot_description_launch_path),
    #         launch_arguments={
    #             'use_sim_time': str(use_sim_time),
    #             'publish_joints': 'true',
    #         }.items()
    #     ),

    pkg_path = get_package_prefix('jackal_description')
    model_path = os.path.join(pkg_path, "share")
    resource_path = pkg_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']
    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        declare_world_name,
        spawn_robot,
        launch_jackal_control,
        launch_jackal_teleop_base,
        launch_jackal_description])


    # Create the launch description and populate
    # ld = LaunchDescription([gz_resource_path, spawn_robot, launch_jackal_control, launch_jackal_description, launch_jackal_teleop_base. declare_world_name])

    # ld.add_action(declare_gz_pose)
    # ld.add_action(gz_resource_path)
    # ld.add_action(declare_model_name)
    # ld.add_action(spawn_robot)
    # ld.add_action(launch_jackal_control)
    # ld.add_action(launch_jackal_description)
    # ld.add_action(launch_jackal_teleop_base)

    # return ld