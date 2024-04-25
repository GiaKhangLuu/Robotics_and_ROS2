#!/usr/bin/env python3

# To start the script, we need to declare the python interpreter directory. Since 
# CMake build type that we used cannot automatically locate it, so we have to 
# where it located

import rclpy 
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AngleConverter(Node):
    def __init__(self):
        super().__init__("angle_conversion_service_server")

        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternionToEulerCallback)
        self.get_logger().info("Angle Conversion Services are Ready")

    def eulerToQuaternionCallback(self, req, res):
        self.get_logger().info("Requested to convert euler angles roll: %f, pitch: %f, yaw: %f, into a quaternion" % (req.roll, req.pitch, req.yaw))
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info("Corresponding quaternion x: %f, y: %f, z: %f, w: %f" % (res.x, res.y, res.z, res.w))
        return res

    def quaternionToEulerCallback(self, req, res):
        self.get_logger().info("Requested to convert quaternion x: %f, y: %f, z: %f, w: %f" % (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info("Corresponding euler angles roll: %f, pitch: %f, yaw: %f" % (res.roll, res.pitch, res.yaw))
        return res

def main():
    rclpy.init()
    angle_converter = AngleConverter()
    rclpy.spin(angle_converter)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

"""
We need to inform the compiler about the existence of this python node so that
it can be compiled and converted into an executable.

Since we are using CMake as a build type for this `arduinobot_utils` package, 
we need to follow a slightly different approach for the installation of a 
node using the CMakeLists.txt file, where we can install both python and C++ 
node.
"""