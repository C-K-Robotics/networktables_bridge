// Copyright (c) 2025 C.K. Robotics

#ifndef NETWORKTABLES_COMMON__TEST_NT_COMMON_HPP_
#define NETWORKTABLES_COMMON__TEST_NT_COMMON_HPP_

#include <memory>
#include <vector>
#include <cmath>
#include <math.h>

#include "networktables_common/nt_pubsub.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"


namespace nt
{

class TestNTCommonNode : public rclcpp::Node
{
public:
  explicit TestNTCommonNode(const rclcpp::NodeOptions & options);

  ~TestNTCommonNode() {
    inst_.RemoveListener(conn_listener_handle_);
  }

private:  
  // Subscribers and publishers
  rclcpp::TimerBase::SharedPtr step_timer_20_hz_;
  rclcpp::TimerBase::SharedPtr step_timer_100_hz_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr data_1_copy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr my_name_copy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_1_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_2_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_name_pub_;

  nt::NetworkTableInstance inst_;
  NT_Listener conn_listener_handle_;

  pubsub::TopicSubscriber<DoubleTopic>::UniquePtr double_topic_subscriber_1_;
  pubsub::TopicSubscriber<DoubleTopic>::SharedPtr double_topic_subscriber_2_;
  pubsub::TopicSubscriber<StringTopic, TestNTCommonNode>::SharedPtr string_topic_subscriber_;
  pubsub::TopicPublisher<DoubleTopic>::SharedPtr double_topic_publisher_1_;
  pubsub::TopicPublisher<StringTopic>::SharedPtr string_topic_publisher_;

  void step_20_hz();
  void step_100_hz();
  void on_data_1_copy_received(const std_msgs::msg::Float64::SharedPtr msg);
  void on_my_name_copy_received(const std_msgs::msg::String::SharedPtr msg);
  void on_string_topic_received(const std::shared_ptr<StringTopic::ValueType> msg);
};

}  // namespace nt

#endif  // NETWORKTABLES_COMMON__TEST_NT_COMMON_HPP_