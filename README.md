2024/03/25: Starting to study Robotics and Ros2.

2024/04/10: Nearly finish the course.


# Commands 

There are some commands we need to know in ROS2.

```
colcon build
```

`colcon` is a build tool that converts the source code and the scripts, the one 
that we are going to develop in `C++` or `python` or any other programming language into  executable files that can be run, that can be executed as normal applications within the system.

```
ros2 pkg create --build-type ament_cmake your_package_name
```

Creating a package. Using `ament_python` if we use `python` for programming.

```
. install/setup.bash
```

Activating our workspace. This way, all the packages that we added and that we are 
going to add in this workspace will be recognized as ROS2 packages and so we will
be able to execute their content, their script with the ROS2 commands.

```
ros2 run your_package_name your_node_name
``` 

Starting a node.

```
ros2 topic list
```

Showing all the topics that are currently available in ROS2.


```
ros2 topic echo /your_topic_name
```

Seeing which messages are now travelling in the topic.

```
ros2 topic info /your_topic_name
```

Seeing the additional information about a topic.

```
ros2 topic info /your_topic_name --verbose
```

Getting more information about a topic.

```
ros2 topic hz /your_topic_name
```

This command analyzes the messages that are published in the topic and calculates
the frequency of the publishing.

```
ros2 topic pub /your_topic_name type_of_a_message message_content
```

Publish a new message to a specific topic.

```
ros2 launch urdf_tutorial display.launch.py model:=abs/path/to/your/urdf/model
```

Display our URDF model.

```
ros2 param list
```

Provide a list of all the parameters that are available for each node.

```
ros2 param get name_of_node name_of_parameter
```

See the value of parameter of a specified node.

```
ros2 run your_package_name your_node_name --ros-args -p name_of_parameter:=value_of_parameter
```

Start a specified node and set value for its parameter.

```
ros2 param set name_of_node name_of_parameter new_value_of_that_parameter
```

Update value of a parameter of a specified node.

```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro abs/path/to/your/xacro/urdf/model)"
```

Publish the URDF model within ROS2 topic, convert xacro urdf model into a plain urdf model and pass to the robot_description parameter.
This node publishes the position and the orientation of all the links of the robot.

```
ros2 run joint_state_publisher_gui joint_state_publisher_gui 
```

Start a graphical interface with some sliders which corresponds to all movable joints of our robot which we can use then to move
around our URDF model. 

```
ros2 run rviz2 rviz2 
```

Launch RVIZ.

```
ros2 launch name_of_the_package name_of_the_launch_file
```

Start the launch file.

```
ros2 control list_controllers
```

Obtain a list of all the currently configured and active controllers for our robot. 

```
ros2 control list_hardware_components
```

Obtain a list of the hardware components that are currently available and that are
currently configured to work with the ROS2 control interface.

```
ros2 run tf2_tools view_frames
```

Run `view_frames` node from the `tf2_tools` library that allows us to graphically
visualize the frames and their connection in a tree structure. 

```
ros2 run tf2_ros tf2_echo name_of_ref_frame_1 name_of_ref_frame_2
```

Display the transformation matrix between any two frames.

```
ros2 service list
```

List all the current availabel services

```
ros2 service type name_of_a_service
```

Get a type of the communication interface that the service `name_of_a_service` is using.

```
ros2 service call name_of_a_service type_of_message_interface content_of_the_message
```

Send a request message to the service `name_of_a_service`.

```
ros2 action list
```

List all of the currently availabel actions in ROS2.

```
ros2 action info name_of_action_server -t
```

Get the information about the action server called `name_of_action_server`. The flag `-t` used 
to display also the type of the message interface to communicate with this action server.

```
ros2 action send_goal name_of_action_server interface_message content_of_the_message -f
```

Send goal to the action server. The flag `-f` used to display the feedback.