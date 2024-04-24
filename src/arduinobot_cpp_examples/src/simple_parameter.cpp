/*
Implementing a simple C++ node that can be configured with 
parameters and change its behavior based on the assigned
values to those parameters.

Alan: 2024/04/23
*/
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <vector>
#include <memory>

using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{
public:
	SimpleParameter() : Node("simple_parameter")  // a node named `simple_parameter`
	{
		/*
		Declare a new node configuration parameter using declare_parameter function.
		Inside the angular brackets is the type of the parameter bc this is template function.
		Inside the parentheses brackets is the name and default value of that parameter.	
		*/
		declare_parameter<int>("simple_int_param", 28);
		declare_parameter<std::string>("simple_string_param", "Antonio");

		/*
		add_on_set_parameters_callback function will be executed whenever one or more
		parameters that are declared in the node are changed.	
		We have one argument in the callback function. Therefore, placeholder is _1.
		*/
		param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
	}

private:
	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	/*
	This is our callback function, this function will be executed whenever our 
	parameters are changed.	
	*/
	rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
	{
		rcl_interfaces::msg::SetParametersResult result;
		for(const auto& param : parameters)
		{
			if(param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
			{
				RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed! New value is: " << param.as_int());
				result.successful = true;
			}
			if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
			{
				RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! New value is: " << param.as_string());
				result.successful = true;
			}
		}

		return result;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SimpleParameter>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}