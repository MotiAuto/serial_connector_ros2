#include "serial_connector_ros2.hpp"

namespace serial_connector_ros2
{
    SerialConnectorROS2::SerialConnectorROS2(const rclcpp::NodeOptions& options):rclcpp::Node("serial_connector_ros2", options)
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/write",
            0,
            std::bind(&SerialConnectorROS2::topic_callback, this, _1));

        this->declare_parameter("port_name", "/dev/ttyACM0");
        this->get_parameter("port_name", port_name_);

        this->declare_parameter("baud_rate", 115200);
        this->get_parameter("baud_rate", baud_rate_);

        handler_ = std::shared_ptr<serial_connector_ros2::SerialHandler>(serial_connector_ros2::SerialHandler::init_handler(port_name_, baud_rate_));

        int open_err = handler_->open_port();
        if(open_err < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Port");
            handler_->close_port();
        }

        RCLCPP_INFO(this->get_logger(), "Start SerialConnectorROS2 port:%s, baud_rate:%d", handler_->get_port_name().c_str(), handler_->get_baud_rate());
    }

    void SerialConnectorROS2::topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string tx_packet;
        tx_packet = msg->data;

        int write_err = handler_->write_serial(tx_packet);

        if(write_err < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Write %s", tx_packet.c_str());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(serial_connector_ros2::SerialConnectorROS2)