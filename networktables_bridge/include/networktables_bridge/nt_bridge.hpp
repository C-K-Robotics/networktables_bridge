// Copyright (c) 2025 C.K. Robotics

#ifndef NETWORKTABLES_BRIDGE__NT_BRIDGE_HPP_
#define NETWORKTABLES_BRIDGE__NT_BRIDGE_HPP_

#include <memory>
#include <vector>
#include <cmath>
#include <math.h>

#include "networktables_common/nt_pubsub.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "frc_msgs/msg/misc_report.hpp"

namespace nt
{

class NTBridgeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NTBridgeNode(const std::string & name, const rclcpp::NodeOptions & options);

  ~NTBridgeNode() {
    inst_.RemoveListener(conn_listener_handle_);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

private:  
  // Subscribers and publishers
  rclcpp::TimerBase::SharedPtr step_timer_50_hz_;
  rclcpp_lifecycle::LifecyclePublisher<frc_msgs::msg::MiscReport>::SharedPtr misc_report_pub_;

  nt::NetworkTableInstance inst_;
  NT_Listener conn_listener_handle_;

  // std::vector<std::shared_ptr<pubsub::TopicSubscriber>> system_stats_subscribers_;

  void step_50_hz();
  // void on_data_1_copy_received(const std_msgs::msg::Float64::SharedPtr msg);
  // void on_my_name_copy_received(const std_msgs::msg::String::SharedPtr msg);
  // void on_string_topic_received(const std::shared_ptr<StringTopic::ValueType> msg);
};

}  // namespace nt

#endif  // NETWORKTABLES_BRIDGE__NT_BRIDGE_HPP_