"""
So far, everything is ready to start the control system of our 
simulated robot in gazebo. The URDF model is configured to 
load the gazebo ros2 control plugin and the interface of the
simulated robot gazebo with the ROS2 control library. 

We have defined the input and the output interfaces of each 
joint of the robot and also we have configured the controller
manager to interact with the robot's hardware and also with 
the other ROS2 applications.

Now we will create a new launch file that starts all of this
logic and functionalities.

Alan Luu: 2024/04/25.
"""

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

    # Launching the robot_state_publisher node which reads the URDF model and publishes its information
    # into our ROS2 topic.
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

    # This is our new ROS2 node that we want to start from the package called `controller_manager`.
    joint_state_broadcaster_spawner = Node(  
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",  # the name that we set for the controller, this means that we want the `controller_manager` to spawn the `joint_state_broadcaster` controller 
            "--controller-manager",
            "/controller_manager"  # start it with the namespace  is `controller_manager`
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

"""
We have created the new launch folder within the `arduinobot_controller` package, we need
to inform the compiler about the existence of this folder and also that it should install 
it.
"""