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
