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

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# ARGUMENTS = [
#     DeclareLaunchArgument(
#         "world_path", default_value="", description="The world path, by default is empty.world"
#     ),
# ]

def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=[ #same
                                                EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                                                "/usr/share/gazebo-11/models/:",
                                                str(Path(get_package_share_directory("jackal_description")).parent.resolve()),
    ])

    # Launch args #same
    # world_path = LaunchConfiguration('world_path')
    # prefix = LaunchConfiguration('prefix')

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
        # launch_arguments={"robot_description_command": robot_description_command}.items(),
        launch_arguments=[("robot_description_command", robot_description_command)],
        # launch_arguments={'robot_description_command':robot_description_command, 'is_sim':'True',}.items()
    )

    #gazebo server, client in example_cafe2.launch.py
    # declare_model_name = DeclareLaunchArgument(
    #     'model_name', default_value='jackal',
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
                [FindPackageShare("jackal_control"), "launch", "hunav_control.launch.py"])
        ),
        # launch_arguments={'robot_description_command':robot_description_command, 'is_sim':'True',}.items()
        launch_arguments=[
            ("robot_description_command", robot_description_command),
            ("is_sim", "True")]
        # launch_arguments={"is_sim": "true"}.items(),

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


    # Create the launch description and populate
    ld = LaunchDescription([gz_resource_path, spawn_robot, launch_jackal_control, launch_jackal_description, launch_jackal_teleop_base])

    # ld.add_action(declare_gz_pose)
    # ld.add_action(gz_resource_path)
    # ld.add_action(declare_model_name)
    # ld.add_action(spawn_robot)
    # ld.add_action(launch_jackal_control)
    # ld.add_action(launch_jackal_description)
    # ld.add_action(launch_jackal_teleop_base)

    return ld