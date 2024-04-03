import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(get_package_share_directory('arduinobot_description'), "urdf", "arduinobot.urdf.xacro")
        ]), 
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    """
    For each of the controllers that we have created (arm_controller, gripper_controller and joint_state_broadcaster) 
    and configured for the `controller manager` inside the `configuration (.yaml) file`, we need to create 
    a new node that starts each of those controller => We need to start three nodes.
    """

    # This is our new ROS2 node that we want to start from the package controller manager
    joint_state_broadcaster_spawner = Node(  
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Spawning `arm_controller` from the `controller_manager`
    arm_controller_spawner = Node(  
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Spawning `gripper_controller` from the `controller_manager`
    gripper_controller_spawner = Node(  
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])