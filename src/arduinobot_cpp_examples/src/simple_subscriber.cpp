/*
Implementing a subscriber node in C++ that use the publisher 
subscriber communication protocol to receive and read all 
the messages that pass through a topic.

Alan: 2024/04/11.
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
	SimpleSubscriber() : Node("simple_subscriber") // the name of this node is `simple_subscriber`
	{	
		/*
		Create a new subscriber object using the create_subscription(), this 
		subscriber node receives messages which type is String, and also 
		this node listens to the `chatter` topic.
		The callback function receives only one arguments, so we have one placeholder.
		*/
		sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1));
	}

	void msgCallback(const std_msgs::msg::String &msg) const
	{
		RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
	}

private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);  // Initialize ROS2 
	auto node = std::make_shared<SimpleSubscriber>();  // Create a pointer to an object of the SimpleSubscriber class
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}

/*
To execute this node. Firstly, we need to instruct the compiler on how it should build 
our script and make it executable in ROS2.
*/