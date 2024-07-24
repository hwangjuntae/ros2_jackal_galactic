from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    #ros1 control.yaml, robot_localization.yaml(ekf), twist_mux.yaml, joint_position_controller.yaml
    # Configs
    config_jackal_ekf = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
        'config',
        'localization2.yaml'],
    )

    config_imu_filter = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
        'config',
        'imu_filter.yaml'],
    )

    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
        'config',
        'control2.yaml'],
    )

    # Launch Arguments
    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal_base.urdf.xacro']
            )
        ]
    )

    is_sim = LaunchConfiguration('is_sim', default=True) #바꿔주었지만 의미가 있는건지 모르겠음

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value=is_sim)

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    # Localization
    localization_group_action = GroupAction([
        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_jackal_ekf],
        ),

        # Madgwick Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[config_imu_filter]
        )
    ])

    # ROS2 Controllers
    control_group_action = GroupAction([
        # ROS2 Control
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content},
                        config_jackal_velocity_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            condition=UnlessCondition(is_sim)
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',  
            arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager"],
            output='screen',
        ),

        # Velocity Controller
        Node(
            package='controller_manager',
            executable='spawner', 
            arguments=['jackal_velocity_controller', "--controller-manager", "/controller_manager"],
            output='screen',
        )
    ])

    ld = LaunchDescription()
    ld.add_action(robot_description_command_arg)
    ld.add_action(is_sim_arg)
    ld.add_action(localization_group_action)
    ld.add_action(control_group_action)
    return ld