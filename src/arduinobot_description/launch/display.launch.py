"""
Create our first launch file to start visualizing URDF
model of the robot in RViz with just one terminal and
just one command to start the launch file.

Alan: 2024/04/24.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

"""
This function is executed when we launch the launch file. This 
function return a LaunchDescription object that contains a 
list of all the applications, so of all the nodes that we 
want to start.
"""
def generate_launch_description():

	"""
	Declare an argument for this launch file which can be changed when this launch 
	file is started. For example, we can use this same launch file to visualize 
	also other URDF model of other robot that we might create in the future.
	"""
	model_arg = DeclareLaunchArgument(
		name="model",  # name of this argument
		default_value=os.path.join(get_package_share_directory("arduinobot_description"), 'urdf', 'arduinobot.urdf.xacro'),  # default value of this argument
		description="Absolute path to the robot URDF file"  # description of this argument
	)

	"""
	This variable contains the value of the `robot_description` parameter for the `robot_state_publisher` node.
	Our URDF is in xacro format. Therefore, we first need to convert this xacro format into plain URDF format
	by run Command(). 
	The path of our URDF model is specified by the argument named `model` which is defined above.
	"""
	robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration('model')]))

	"""
	Initialize the `robot_state_publisher` node which is the one that reads the URDF model of the robot 
	and publish it within our ROS2 topic.
	"""
	robot_state_publisher = Node(
		package = "robot_state_publisher",  # specify the package that contains the node that we want to start 
		executable = "robot_state_publisher",  # specify the name of the node that we want to start
		parameters = [{"robot_description": robot_description}]  # configure this node with some parameters. Specifically, we need to indicate where the URDF model is located
	)

	"""
	Initialize the `joint_state_publisher_gui` node that offers the sliders of movable joints.
	"""
	joint_state_publisher_gui = Node(
		package='joint_state_publisher_gui',
		executable = 'joint_state_publisher_gui'
	)


	"""
	Initialize the `rviz` node.
	"""
	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',  # see the output of this node in the terminal
		arguments=['-d', os.path.join(get_package_share_directory('arduinobot_description'), 'rviz', 'display.rviz')]  # load the display.rviz by default when we start rviz
	)

	"""
	Now we have declared an argument and three nodes, we can pass these variables to this
	launch description file.
	"""
	return LaunchDescription([
		model_arg,
		robot_state_publisher,
		joint_state_publisher_gui,
		rviz_node
	])

"""
Now we have finished this launch file. Next, we need to install the launch folder which we 
created inside the CMakeLists.txt.
"""