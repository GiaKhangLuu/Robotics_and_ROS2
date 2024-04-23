/*
Implementing a simple C++ node that listen for 
messages coming from the Arduino through the
serial port and republish each new message 
that is received from the Arduino into a 
ROS2 topic.

Alan: 2024/04/11.
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

#include <chrono>  

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
public:
	SimpleSerialReceiver() : Node("simple_serial_receiver")
	{
        declare_parameter<std::string>("port", "/dev/ttyACM0");	
        std::string port_ = get_parameter("port").as_string();

		pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10);   
		timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timerCallback, this));

        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
	}

	void timerCallback()
	{   
		auto message = std_msgs::msg::String();
        if(rclcpp::ok() && arduino_.IsDataAvailable())
        {
            arduino_.ReadLine(message.data);
        }
		pub_->publish(message);  
	}

private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;  
	rclcpp::TimerBase::SharedPtr timer_; 
    LibSerial::SerialPort arduino_;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv); 
	auto node = std::make_shared<SimpleSerialReceiver>();  
	rclcpp::spin(node);  
	rclcpp::shutdown();  

	return 0;    
}

/*
Now we can proceed to execute this node. But first we need to instruct the compiler on how 
it should build our script and make it executable in ROS2.
*/