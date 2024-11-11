#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

#include "math.h"

class Cpp_Node_Cluster : public rclcpp::Node
{
public:
    Cpp_Node_Cluster()
        : Node("cpp_node_cluster", options)
    {
        // Create a publisher on the output topic
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // Create a timer that calls the publish_message method every 500 ms
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Cpp_Node_Cluster::publish_message, this));
    }

private:
    void publish_message()
    {
        // Create a message to send
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        // Publish the message
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};