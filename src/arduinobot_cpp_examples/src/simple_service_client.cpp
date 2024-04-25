/*
Create a service client in ROS2 using C++ that use the functionality 
offered by the service server. The client will send a request to the
service according to the message interface defined by the service 
and then will be notified with a response message when the server
completes its execution.

Alan Luu: 2024/04/25.
*/

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/add_two_ints.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleServiceClient : public rclcpp::Node
{
public:

    /* 
    When we are going to create a new object of the SimpleServiceClient class, we 
    need to pass the two number a and b that we want to send so that we want to 
    request to the service server.
    */
    SimpleServiceClient(int a, int b) : Node("simple_service_client")
    {
        /*
        create_client() is a template function that helps to create service client.
        Inside the angular brackets is a type of the interface used for communication.
        Inside the parenthese brackets is a name of the service. 
        */
        client_ = create_client<arduinobot_msgs::srv::AddTwoInts>("add_two_ints");
        auto request = std::make_shared<arduinobot_msgs::srv::AddTwoInts::Request>();  // create an instance of the request interface
        request->a = a;
        request->b = b;

        /*
        Before sending these requests to the server, let's first verify that actually the server is
        running and so it's availabel to receive new requests. 
        */
        while (!client_->wait_for_service(1s))  // Let the client to wait for the service 1s before considering it not available
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, waiting more time...");
            if (!rclcpp::ok())
            {
                // If ROS is not running anymore, let's terminate the execution.
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service");
                return;
            }
        }
        
        /*
        If the code now exit the while loop, that means that we have found a service named `add_two_ints` in 
        ROS2 and also that this service is ready to process any new requests. 
        async_send_request() helps us send request to server without waiting for the server to finish its 
        execution.
        When the service finish its execution, the callback function is called.
        */
        auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, 
                                                                     this,
                                                                     _1));
    }

private:

    // Declare the type of the interface that it needs for the communication with the server which is `AddTwoInts` type.
    rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Service Response " << future.get()->sum);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service Failure");
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 3) // Here we check whether the number of arguments is equal to 3 because we must have 3 arguments: name_of_the_node, number a and number b
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong number of arguments! Usage: simple_service_client A B");
        return 1;
    }

    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));  // atoi helps us convert char to int
    rclcpp::shutdown();

    return 0;
}

/*
So far, the simple service client is completed, so we can proceed to 
declare it and install it within the CMakeLists.txt file.
*/