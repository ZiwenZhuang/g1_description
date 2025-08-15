import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    publish_fake_state_arg = DeclareLaunchArgument("publish_fake_states", default_value='false')
    robot_model_arg = DeclareLaunchArgument("robot_model", default_value="29")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("g1_description"), "config", "check_joints.rviz"],
    )

    def process_arguments(context, *args, **kwargs):
        # Access the parameter value
        robot_model_ = context.launch_configurations.get('robot_model', "29")

        bringup_dir = get_package_share_directory("g1_description")
        urdf_path = os.path.join(bringup_dir, "urdf", f"g1_{robot_model_}dof_torsobase.urdf")
        g1_description = open(urdf_path).read()

        robot_state_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": g1_description,
            }],
        )
        return [robot_state_node]

    return LaunchDescription([
        publish_fake_state_arg,
        robot_model_arg,
        OpaqueFunction(function=process_arguments),
        Node(
            package= "rviz2",
            executable= "rviz2",
            name= "rviz2",
            arguments= [
                "-d",
                rviz_config_file,
            ],
        ),
    ])
