# This file acts as a sort of instruction sheed that tells the compiler
# how it should translate our C++ scripts into executable files.

cmake_minimum_required(VERSION 3.8)
project(arduinobot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Firstly, before building our script and making them executable. We
# need to declare the support packages that we use the dependencies 
# that we use to develop the functionalities of our node
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(arduinobot_msgs REQUIRED)  # the package that contains the definition of the interface with the service
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
#find_package(rclcpp_lifecycle REQUIRED)

# Now we can proceed to instruct the compiler on how it should build
# and install our script
add_executable(simple_publisher src/simple_publisher.cpp)
# Declare the dependencies to this executable.
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp rcl_interfaces)

add_executable(simple_service_server src/simple_service_server.cpp)
ament_target_dependencies(simple_service_server rclcpp arduinobot_msgs)

add_executable(simple_service_client src/simple_service_client.cpp)
ament_target_dependencies(simple_service_client rclcpp arduinobot_msgs)

add_executable(simple_moveit_interface src/simple_moveit_interface.cpp)
ament_target_dependencies(simple_moveit_interface rclcpp moveit_ros_planning_interface)

add_library(simple_action_server SHARED src/simple_action_server.cpp)  # add a new library that is called simple_action_server
target_include_directories(simple_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(simple_action_server
  PRIVATE "SIMPLE_ACTION_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(simple_action_server  # add dependencies of the library
  "arduinobot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(simple_action_server  # registering that implements the action server which is the one that is in the arduinobot_cpp_exmaples namespace and is implemented in the class SimpleActionServer
  PLUGIN "arduinobot_cpp_examples::SimpleActionServer"
  EXECUTABLE simple_action_server_node
)

add_library(simple_action_client SHARED src/simple_action_client.cpp)
target_include_directories(simple_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(simple_action_client 
  PRIVATE "SIMPLE_ACTION_CLIENT_CPP_BUILDING_DLL")
ament_target_dependencies(simple_action_client 
  "arduinobot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(simple_action_client 
  PLUGIN "arduinobot_cpp_examples::SimpleActionClient"
  EXECUTABLE simple_action_client_node
)

#add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
#ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

# Install this script
install(TARGETS  # specify the executable that we want to install
  simple_publisher
  simple_subscriber
  simple_parameter
  simple_service_server
  simple_service_client
  simple_moveit_interface
  #simple_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}  # specify where we want to install the executable above
)

install(TARGETS
  simple_action_server
  simple_action_client 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Before building the workspace, we need also to declare the dependencies that we use
