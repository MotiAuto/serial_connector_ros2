#ifndef SERIAL_CONNECTOR_ROS2_HPP_
#define SERIAL_CONNECTOR_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


#include "serial_utils.hpp"

using std::placeholders::_1;

namespace serial_connector_ros2
{
    class SerialConnectorROS2 : public rclcpp::Node
    {
        public:
        explicit SerialConnectorROS2(const rclcpp::NodeOptions&options = rclcpp::NodeOptions());

        private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        std::shared_ptr<serial_connector_ros2::SerialHandler> handler_;
        std::string port_name_;
        int baud_rate_;
    };
}

#endif