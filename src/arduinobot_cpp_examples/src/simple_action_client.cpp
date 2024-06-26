#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arduinobot_msgs/action/fibonacci.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace arduinobot_cpp_examples
{
class SimpleActionClient : public rclcpp::Node
{
    public:
        explicit SimpleActionClient(const rclcpp::NodeOptions& options) : Node("simple_action_client", options)
        {
            /*
            create_client() function takes two inputs.
            1: The instance of ROS2 Node that contains the action client. So actually this is the current object, so we set `this`. 
            2: The name of the action server.
            */
            client_ = rclcpp_action::create_client<arduinobot_msgs::action::Fibonacci>(
                this,
                "fibonacci"
                );
            timer_ = create_wall_timer(1s, 
                                       std::bind(&SimpleActionClient::timerCallback, this));
        }
    
    private:
        rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_; // this object repeatedly execute a certain function at regular time intervals

        void timerCallback()
        {
            // We dont need this function to be executed repeatedly every second, we need it just to be executed once.
            // So just the first time, so whenever we execute it for the first time, let's cancel it so that it will
            // not be executed again => This callback function is executed only once.
            timer_->cancel();

            /* Verifying that the action server is running */
            if (!client_->wait_for_action_server())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            /* Sending goal message */
            auto goal_msg = arduinobot_msgs::action::Fibonacci::Goal();
            goal_msg.order = 10;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending Goal");

            auto send_goal_options = rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SendGoalOptions();

            /* 
            Now we can use this `send_goal_options` class to define a series
                of callback functions that will be executed when some event 
                occurs during the interaction between the client and the server.
            */

            /* 
            With the `goal_response_callback`, we can define the function
                so the behavior that the client will execute after sending
                the goal to the actoin server. 
            */
            send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goalCallback, 
                                                                 this,
                                                                 _1);
            /* 
            The `feedback_callback` function that is the function that will
                be executed everytime the action server sends a feedback
                message to the client to inform it about the progress of
                the action. 
            */
            send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback,
                                                            this,
                                                            _1,
                                                            _2);

            /*
            The `result_callback` will be executed when the action server 
                terminates its execution and returns result message to 
                the clien. 
            */
            send_goal_options.result_callback = std::bind(&SimpleActionClient::resultCallback,
                                                          this,
                                                          _1);     

            /* Finally, send the goal to the action server */
            client_->async_send_goal(goal_msg, send_goal_options);
        }

        void goalCallback(const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr& goal_handle)
        {
            /* 
            Check whether of not the goal has been 
                accepted by the action server 
            */
            if (!goal_handle)
            {
                /* The goal has not been accepted */
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by the server");
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by the server, waiting for result");
            }
        }

        void feedbackCallback(rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr,
                              const std::shared_ptr<const arduinobot_msgs::action::Fibonacci::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Next number in sequence received: ";
            for (auto number : feedback->partial_sequence)
            {
                ss << number << "";
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
        }

        void resultCallback(const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::WrappedResult& result)
        {
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
                    return;
            }

            std::stringstream ss;
            ss << "Next number in sequence received: ";
            for (auto number : result.result->sequence)
            {
                ss << number << "";
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
            rclcpp::shutdown();
        }

};
} 

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionClient)

/*
Now we can instruct the compiler on how it should build and make this
    is a C++ script and executable.
*/