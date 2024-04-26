/*
Create a simple action server in C++ that calculates the Fibonacci sequence. 

Alan Luu: 2024/04/26.
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arduinobot_msgs/action/fibonacci.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <thread>

using namespace std::placeholders;

namespace arduinobot_cpp_examples
{
    
class SimpleActionServer : public rclcpp::Node
{
    public:
        explicit SimpleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("simple_action_server", options)
        {   
            /*
            To create server, we need total 5 arguments:
            1. The instance of ROS2 Node that hosts the action server. This is the current node so we pass `this`. 
            2. The name of the action server.
            3. Callback function 01 which is called when the action server receives a new goal from action client. 
            4. Callback function 02 which is called when the action server cancels a goal from action client.
            5. Callback function 03 which is called when the action server accepts a goal from action client. 
            */
            action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::Fibonacci>(
                this, "fibonacci", std::bind(&SimpleActionServer::goalCallback, this, _1, _2), 
                std::bind(&SimpleActionServer::cancelCallback, this, _1), 
                std::bind(&SimpleActionServer::acceptedCallback, this, _1)
            );

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting the Action Server");
        }

    private:
        rclcpp_action::Server<arduinobot_msgs::action::Fibonacci>::SharedPtr action_server_;

        rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid, 
                                                 std::shared_ptr<const arduinobot_msgs::action::Fibonacci::Goal> goal)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with order: " << goal->order);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>> goal_handle)
        {
            /*
            Note that we dont wanna keep the client blocked for too long while waiting for the server to execute. So 
            we must place the actual logic of the action server within a different thread in order to avoid to block
            the current thread. And so to allow to return the message to the client as fast as possible so that the
            client can continue with its execution while the server is performing its functionalities. 
            */
            std::thread{std::bind(&SimpleActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");
            rclcpp::Rate loop_rate(1);  // this object can be used to control the execution of a loop within ROS2 node 

            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<arduinobot_msgs::action::Fibonacci::Feedback>();  // we can use this feedback message to send periodically messages to the action client in order to inform it about the progress of the action server
            auto& sequence = feedback->partial_sequence;

            sequence.push_back(0);  // add first element to the sequence
            sequence.push_back(1);  // add second element to the sequence

            auto result = std::make_shared<arduinobot_msgs::action::Fibonacci::Result>();

            for(int i = 1; (i < goal->order) && rclcpp::ok(); i++)
            {
                if(goal_handle->is_canceling())
                {
                    result->sequence = sequence;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Canceled");
                    return;
                }

                sequence.push_back(sequence[i] + sequence[i - 1]);
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publish Feedback");

                loop_rate.sleep();
            }

            if(rclcpp::ok())
            {
                result->sequence = sequence;
                goal_handle->succeed(result);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
            }
        }

        rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel the goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
};
} 

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionServer)