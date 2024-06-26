"""
At this point, Moveit2 is configured and also supports our robot. So now we can finally start 
using its amazing features to move the robot's end effector from the starting point to the
end point. And also to plan and execute trajectories to launch this software with the 
configuration that we have created. 

Now we create a new launch file.

Alan Luu: 2024/04/26.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Declaring a list of instructions that needs to be 
    executed when we start the launch file, so when 
    we start the application
    """

    # `is_sim` argument indicates whether or not we are starting the Moveit functionalities
    # for the simulation in gazebot or for the real robot.
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    # Declare configuration based on the files that we created in the previous lesson.
    moveit_config = (
        MoveItConfigsBuilder("arduinobot", package_name="arduinobot_moveit")
        .robot_description(file_path=os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduinobot.urdf.xacro"))
        .robot_description_semantic(file_path="config/arduinobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()
    # Remove keys with None values
    moveit_config_dict = {key: value for key, value in moveit_config_dict .items() if value is not None}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict, {"use_sim_time": is_sim}, {"publish_robot_description_semantic": True}],
        arguments=["--ros-args", "--log-level", "info"]
    )

    rviz_config = os.path.join(get_package_share_directory("arduinobot_moveit"), "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits]
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node
    ])

"""
Now let's go into the CMakeLists.txt to in order to install the config launch folder, so that
these folders are recognized in ROS2.
"""