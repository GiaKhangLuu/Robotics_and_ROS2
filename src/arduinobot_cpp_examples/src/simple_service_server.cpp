/*
Develop a simple server server in C++ that will provide the 
functionality of calculating the sum of two integers.

Alan Luu: 2024/04/25.
*/

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        /*
        create_service is a template function which used to intialize our service server
        Inside the angular brackets is the type of the interface used for the communication with the service server.
        Inside the parentheses brackers is the name of the service and a callback function which is executed when it receives a new request.
        The callback function takes two arguments so we have _1 and _2 as the placeholders.
        */
        service_ = create_service<arduinobot_msgs::srv::AddTwoInts>("add_two_ints", 
                                                                    std::bind(&SimpleServiceServer::serviceCallback,
                                                                    this,
                                                                    _1,
                                                                    _2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service add_two_ints is Ready");
    }

private:
    rclcpp::Service<arduinobot_msgs::srv::AddTwoInts>::SharedPtr service_;  // `AddTwoInts` is the message interface that we want to use to communicate between client and server

    void serviceCallback(const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Request> req,
                         const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New Request Received a: " << req->a << " b: " << req->b);
        res->sum = req->a + req->b;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Returning sum: " << res->sum);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

/*
With this, we have completed our `simple_service_server` node. So before
executing this node, let's modify also the file CMakeLists.txt in order
to install it. So in order to declare the missing dependencies and also
to install new executable.
*/