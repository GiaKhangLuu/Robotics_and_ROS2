/*
Implementing first ROS2 node in C++ that will use the publisher subscriber 
protocol to publish a message within our ROS2 topic.

Alan: 2024/04/10 
*/

/*
This library will allow us to use all the functionalities of ROS2 
within out script.
*/ 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>  

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
	SimplePublisher() : Node("simple_publisher"), counter_(0)  // constructor, a node called `simple_publisher`.
	{
		/*
		Create a new publisher object, here the message type is String.
		Also set the name of the `topic` in which we want the messages
		to be published, now name it `chatter`, and the size of the
		message queue is 10.
		*/
		pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);   

		/*
		Create `timer_` object using `create_wall_timer`. Assuming that 
		we want to execute it with a frequency of `1s` (1 second)
		*/
		timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));

		RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");  // print informative message in the terminal
	}

	void timerCallback()
	{
		/*
		The goal of this function is to publish a new message within the `chatter` topic every time
		this function is executed	
		*/
		auto message = std_msgs::msg::String();
		message.data = "Hello ROS 2 - counter: " + std::to_string(counter_++);
		pub_->publish(message);  // publish the message we have just created
	}

private:
	unsigned int counter_; // count the num of messages that are published within ROS2 topic.
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;  // declare an object of the C++ publisher class, here the message type is String
	rclcpp::TimerBase::SharedPtr timer_; // this attribute `timer_` is a ROS2 timer whose task is to repeatedly execute a specific function at a certain frequency
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv); // initialize ROS2
	auto node = std::make_shared<SimplePublisher>();  // create a pointer to an object of the `SimplePublisher` class
	rclcpp::spin(node);  // keep this node up and running, so that they can continuously send messages with the `chatter` topic
	rclcpp::shutdown();  // destroy the `SimplePublisher` node

	return 0;    
}

/*
Now we can proceed to execute this node. But first we need to instruct the compiler on how 
it should build our script and make it executable in ROS2.
*/