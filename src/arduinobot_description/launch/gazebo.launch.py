import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

	arduinobot_description = get_package_share_directory('arduinobot_description')
	arduinobot_description_prefix = get_package_prefix('arduinobot_description')

	model_path = os.path.join(arduinobot_description, 'models')
	model_path += pathsep + os.path.join(arduinobot_description_prefix, 'share')

	"""
	In order to start gazebo, we need to set an environmental variables in 
	Linux so that gazebo can properly load and can properly visualize the 
	URDF model of our robot.
	"""
	env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)


	model_arg = DeclareLaunchArgument(
		name="model", 
		default_value=os.path.join(arduinobot_description, 'urdf', 'arduinobot.urdf.xacro'),
		description="Absolute path to the robot URDF file"
	)

	robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration('model')]), value_type=str)

	robot_state_publisher_node = Node(
		package = "robot_state_publisher",
		executable = "robot_state_publisher",
		parameters = [{"robot_description": robot_description}]
	)

	"""
	In fact, the gazebo physics engine has two seperate modules:
		1. A server where objects are simulated and also where 
		the calculation for the interaction with the 
		environment are performed.
		2. A client which is just the GUI that allows the user
		to visualize the objects that are currently simulated
		in the server. 
	"""

	start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
		get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py'
	)))

	start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
		get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'
	)))

	"""
	start_gazebo_server and start_gazebo_client will start an empty gazebo server and 
	gazebo client which will visualize all the objects that are simulated in the 
	server. 
	Currently, however, there are no objects simulated in the gazebo environment. So
	let's start by spawning a new object and let's spawn our robot.
	"""

	spawn_robot = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-entity', 'arduinobot', '-topic', 'robot_description'],
		output='screen'
	)

	return LaunchDescription([
		env_variable,
		model_arg,
		robot_state_publisher_node,
		start_gazebo_server,
		start_gazebo_client,
		spawn_robot
	])