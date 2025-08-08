import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bringup_dir = get_package_share_directory("g1_description")
    urdf_path = os.path.join(bringup_dir, "urdf", "g1_29dof_torsobase.urdf")
    g1_description = open(urdf_path).read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("g1_description"), "config", "check_joints.rviz"],
    )

    return LaunchDescription([
        Node(
            package= "robot_state_publisher",
            executable= "robot_state_publisher",
            name= "robot_state_publisher",
            parameters= [{
                "robot_description": g1_description,
            }]
        ),
        # Node(
        #     package= "tf2_ros",
        #     executable= "static_transform_publisher",
        #     name= "static_transform_publisher",
        #     arguments= [
        #         "0.108485", # x
        #         "-0.0175", # y
        #         "0.693171", # z (relative to torso)
        #         "0", # roll
        #         "0.88645", # pitch
        #         "0", # yaw
        #         "base_link",
        #         "d435_sim_depth_link",
        #     ],
        # ),
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
