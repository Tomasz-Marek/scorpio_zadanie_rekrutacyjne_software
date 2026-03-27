#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

namespace recruitment_task {

class IlmeniteCheckNode : public rclcpp::Node {
public:
  explicit IlmeniteCheckNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("ilmenite_check", options),
    publisher_(this->create_publisher<std_msgs::msg::String>(
      "~/output", rclcpp::QoS(10))),
    subscription_(this->create_subscription<std_msgs::msg::String>(
      "~/input", rclcpp::QoS(10),
      std::bind(&IlmeniteCheckNode::input_callback, this, std::placeholders::_1))) {
    RCLCPP_INFO(this->get_logger(), "IlmeniteCheckNode started.");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void input_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received on /ilmenite_check/input: '%s'", msg->data.c_str());

    std_msgs::msg::String output_msg;
    output_msg.data = "Ilmenite check received: " + msg->data;
    publisher_->publish(output_msg);

    RCLCPP_INFO(this->get_logger(), "Published on /ilmenite_check/output: '%s'", output_msg.data.c_str());
  }
};

}  // namespace recruitment_task

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(recruitment_task::IlmeniteCheckNode)
