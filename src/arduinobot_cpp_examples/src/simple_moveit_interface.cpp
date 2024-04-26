/*
Create a simple script, a simple C++ node that use the Moveit2 APIs to send a goal to our
robot and move it to the desired position.

Alan Luu: 2024/04/26.
*/

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    /*
    The `MoveGroupInterface` class allows us to access and to send
        command and also to view the status of a specific move
        group. 
    */
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    /*
    Now we can use `arm_move_group` and `gripper_move_group` to send 
        the goal to the robot. 
    In this case, we just want to send the goal to each of the robot's
        joint specifying the desired position of each joint
    */

    // Desired position for each of the movables joints that belongs to the group 1
    std::vector<double> arm_joint_goal {1.57, 0.0, 0.0};  // This will make rotate just the first join (the base join) by an angle of 1.57rad
    std::vector<double> gripper_joint_goal {-0.7, 0.7};

    /* 
    setJointValueTarget() returns a boolean value indicating if the goal that we
        to the robot is reachable. So it's within the operational limits of the
        robot.
    */ 
    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

    if(!arm_within_bounds | !gripper_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint position were outside the limits");
        return;
    }

    /* 
    Once the statement above is satify, we can finallly request MoveIt to plan and
        execute the trajectory that brings the robot from its current position
        to the desired posion that we indicated above.
    */
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

    /* 
    If true, that means MoveIt was able to correctly plan a path. So a 
        trajectory from the current position to the desired one for 
        our robot.
    */
    bool arm_plan_success = arm_move_group.plan(arm_plan) == 1;  // 1 means success
    bool gripper_plan_success = gripper_move_group.plan(gripper_plan) == 1;  // 1 means success

    if(arm_plan_success && gripper_plan_success)
    {
        // Finally, we can move the robot
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner Succeed, moving the arm and the gripper");
        arm_move_group.move();
        gripper_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed!");
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");

    move_robot(node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

/*
Now this script is completed, we can move on to install this
    node and also instructing the compiler how it should
    build it and make it an executable.
*/