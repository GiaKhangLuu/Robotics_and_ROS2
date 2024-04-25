/*
Use some tools from the TF2 library of ROS2 to create a C++ service
server for the conversion of the orientation of an object from its
representation in quaternion to Euler angles and vice versa.

Alan Luu: 2024/04/25.
*/

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/euler_to_quaternion.hpp>
#include <arduinobot_msgs/srv/quaternion_to_euler.hpp>
#include <tf2/utils.h>

using namespace std::placeholders;

class AnglesConverter : public rclcpp::Node
{
public:
    AnglesConverter() : Node("angles_conversion_service")
    {
        euler_to_quaternion_ = create_service<arduinobot_msgs::srv::EulerToQuaternion>("euler_to_quaternion", 
                                                                                        std::bind(&AnglesConverter::eulerToQuaternionCallback,
                                                                                        this,
                                                                                        _1,
                                                                                        _2));

        quaternion_to_euler_ = create_service<arduinobot_msgs::srv::QuaternionToEuler>("quaternion_to_euler", 
                                                                                        std::bind(&AnglesConverter::quaternionToEulerCallback,
                                                                                        this,
                                                                                        _1,
                                                                                        _2));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle Conversion services are ready");
    }

private:
    rclcpp::Service<arduinobot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_;
    rclcpp::Service<arduinobot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;

    void eulerToQuaternionCallback(const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Request> req,
                                   const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to convert euler angles roll: " << req->roll << ", pitch: " << req->pitch << ", yaw: " << req->yaw << " into a quaternion");
        tf2::Quaternion q;  // create a new instance of the Quaternion class
        q.setRPY(req->roll, req->pitch, req->yaw);
        res->x = q.getX();
        res->y = q.getY();
        res->z = q.getZ();
        res->w = q.getW();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding quaternion x: " << res->x << ", y: " << res->y << ", z: " << res->z << ", w: " << res->w);
    }


    void quaternionToEulerCallback(const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Request> req,
                                   const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to convert quaternion x: " << req->x << ", y: " << req->y << ", z: " << req->z << ", w: " << req->w);
        tf2::Quaternion q(req->x, req->y, req->z, req->w);  // create an object of Quaternion class from the four components of the quaternion that we received in the request message
        tf2::Matrix3x3 m(q);  // create an object of type Matrix3x3, this object is initialized from a Quarternion object
        m.getRPY(res->roll, res->pitch, res->yaw);  // Get the roll, pitch, yaw and store into the response message
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding Euler Angles roll: " << res->roll << ", pitch: " << res->pitch << ", yaw: " << res->yaw);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnglesConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}