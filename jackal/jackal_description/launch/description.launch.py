from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    robot_description_command_arg = DeclareLaunchArgument(
        "robot_description_command",
        default_value=[
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("jackal_description"), "urdf", "jackal_base.urdf.xacro"]
            )
        ]
    )
    
    robot_description_content = ParameterValue(
        Command(LaunchConfiguration("robot_description_command")), 
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}] # "use_sim_time": True
    )
    
    #수정
    # is_sim_arg = DeclareLaunchArgument('is_sim', default_value='True')
    # is_sim = LaunchConfiguration('is_sim')

    # DeclareLaunchArgument(
    #         name='publish_joints', 
    #         default_value='true',
    #         description='Launch joint_states_publisher'
    #     ),


    # robot_joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=IfCondition(LaunchConfiguration("publish_joints")),
    #     parameters=[ #수정
    #         {'use_sim_time':is_sim},{"robot_description": robot_description_content},
    #     ],
    # )

    ld = LaunchDescription()
    # ld.add_action(is_sim_arg) #수정
    ld.add_action(robot_description_command_arg)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(robot_joint_state_publisher_node)

    return ld
