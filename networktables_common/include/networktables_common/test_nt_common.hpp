// Copyright (c) 2025 C.K. Robotics

#ifndef NETWORKTABLES_COMMON__TEST_NT_COMMON_HPP_
#define NETWORKTABLES_COMMON__TEST_NT_COMMON_HPP_

#include <memory>
#include <vector>
#include <cmath>
#include <math.h>

#include "networktables_common/nt_pubsub.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nt
{

class TestNTCommonNode : public rclcpp::Node
{
public:
  explicit TestNTCommonNode(const rclcpp::NodeOptions & options);

  ~TestNTCommonNode() {
    inst_ = nt::NetworkTableInstance::GetDefault();
    inst_.RemoveListener(conn_listener_handle_);
  }

private:  
  // Subscribers and publishers
  rclcpp::TimerBase::SharedPtr step_timer_100_hz_;

  nt::NetworkTableInstance inst_;
  NT_Listener conn_listener_handle_;

  std::unique_ptr<pubsub::TopicSubscriber<DoubleTopic>> double_topic_subscriber_1_;
  std::shared_ptr<pubsub::TopicSubscriber<DoubleTopic>> double_topic_subscriber_2_;
  std::shared_ptr<pubsub::TopicSubscriber<StringTopic, TestNTCommonNode>> string_topic_subscriber_;

  void step_100_hz();
  void on_string_topic_received(const std::shared_ptr<StringTopic::ValueType> msg) {
    RCLCPP_INFO(this->get_logger(), "Yo! I received: %s", msg->c_str());
  }
};

}  // namespace nt

#endif  // NETWORKTABLES_COMMON__TEST_NT_COMMON_HPP_