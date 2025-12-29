// Copyright (c) 2025 C.K. Robotics

#include "networktables_bridge/nt_bridge.hpp"
#include "rcutils/logging_macros.h"

namespace nt
{

NTBridgeNode::NTBridgeNode(const std::string & name, const rclcpp::NodeOptions & options)
: LifecycleNode(name, options)
{
  this->declare_parameter<std::string>("nt_server_ip", "127.0.0.1");
  this->declare_parameter<std::string>("nt_remote_id", name);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NTBridgeNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Misc Report Publisher
  misc_report_pub_ = this->create_publisher<frc_msgs::msg::MiscReport>("misc_report", rclcpp::QoS(10));

  // RCLCPP Timer
  step_timer_50_hz_ =
    rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(0.02), [this] {
      step_50_hz();
    });

  // NetworkTablesInstance Setup
  inst_ = nt::NetworkTableInstance::GetDefault();
  inst_.StartClient4(this->get_parameter("nt_remote_id").as_string());
  inst_.SetServer(this->get_parameter("nt_server_ip").as_string());
  conn_listener_handle_ = inst_.AddConnectionListener(
    true, [this] (const nt::Event& event) {
      if (event.Is(nt::EventFlags::kConnected)) {
        RCLCPP_INFO(this->get_logger(), "Connected to %s\n", event.GetConnectionInfo()->remote_id.c_str());
      } else if (event.Is(nt::EventFlags::kDisconnected)) {
        RCLCPP_INFO(this->get_logger(), "Disconnected from %s\n", event.GetConnectionInfo()->remote_id.c_str());
      }
    }
  );
  // nt::SetNow(this->get_clock()->now().nanoseconds()); // Sync local NT time with ROS2 time

  // Subscribers & Publishers via NetworkTables Common PubSub
  pubsub::subscribe_from(inst_, sys_stats_subscriber_, {{"/AdvantageKit/SystemStats/"}});
  pubsub::subscribe_from(inst_, nt_clients_subscriber_, {{"/AdvantageKit/SystemStats/NTClients/"}});
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NTBridgeNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NTBridgeNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NTBridgeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_50_hz_.reset();
  misc_report_pub_.reset();
  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NTBridgeNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_shutdown(state);
  step_timer_50_hz_.reset();
  misc_report_pub_.reset();

  RCUTILS_LOG_INFO_NAMED(
    get_name(),
    "on shutdown is called from state %s.",
    state.label().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void NTBridgeNode::step_50_hz()
{
  auto team_num_msg = std::static_pointer_cast<int64_t>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/TeamNumber"));
  auto bv_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/BatteryVoltage"));
  auto bc_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/BatteryCurrent"));
  auto v3v3_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/3v3Rail/Voltage"));
  auto c3v3_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/3v3Rail/Current"));
  auto v5v_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/5vRail/Voltage"));
  auto c5v_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/5vRail/Current"));
  auto v6v_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/6vRail/Voltage"));
  auto c6v_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/6vRail/Current"));
  auto cpu_temp_msg = std::static_pointer_cast<double>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/CPUTempCelsius"));
  auto canbus_util_msg = std::static_pointer_cast<float>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/CANBus/Utilization"));
  auto sys_active_msg = std::static_pointer_cast<bool>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/SystemActive"));
  auto rsl_state_msg = std::static_pointer_cast<bool>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/RSLState"));
  auto sys_time_valid_msg = std::static_pointer_cast<bool>(sys_stats_subscriber_->last_received_msg("/AdvantageKit/SystemStats/SystemTimeValid"));

  frc_msgs::msg::MiscReport misc_report_msg;
  misc_report_msg.stamp = this->get_clock()->now();
  if (team_num_msg) misc_report_msg.team_number = static_cast<uint16_t>(*team_num_msg);
  if (bv_msg) misc_report_msg.battery_voltage = static_cast<float>(*bv_msg);
  if (bc_msg) misc_report_msg.battery_current = static_cast<float>(*bc_msg);
  if (v3v3_msg) misc_report_msg.voltage_3v3_system = static_cast<float>(*v3v3_msg);
  if (c3v3_msg) misc_report_msg.amps_3v3_system = static_cast<float>(*c3v3_msg);
  if (v5v_msg) misc_report_msg.voltage_5v_system = static_cast<float>(*v5v_msg);
  if (c5v_msg) misc_report_msg.amps_5v_system = static_cast<float>(*c5v_msg);
  if (v6v_msg) misc_report_msg.voltage_6v_system = static_cast<float>(*v6v_msg);
  if (c6v_msg) misc_report_msg.amps_6v_system = static_cast<float>(*c6v_msg);
  if (cpu_temp_msg) misc_report_msg.rio_cpu_temp = static_cast<float>(*cpu_temp_msg);
  if (canbus_util_msg) misc_report_msg.rio_canbus_utilization = *canbus_util_msg;
  if (sys_active_msg) misc_report_msg.sys_active = *sys_active_msg;
  if (rsl_state_msg) misc_report_msg.rsl_state = *rsl_state_msg;
  if (sys_time_valid_msg) misc_report_msg.sys_time_valid = *sys_time_valid_msg;

  static const std::regex re(
    R"(^/AdvantageKit/SystemStats/NTClients/([^/]+)/Connected$)"
  );

  auto map = nt_clients_subscriber_->last_received_msgs();
  std::smatch match;
  for (const auto& [key, ptr] : map) {
    if (!std::regex_match(key, match, re)) continue;

    std::string client_name = match[1].str();
    frc_msgs::msg::NTClient client_msg;
    client_msg.stamp = this->get_clock()->now();
    client_msg.remote_id = client_name;

    const std::string base = "/AdvantageKit/SystemStats/NTClients/" + client_name + "/";
    if (ptr) client_msg.connected = *static_cast<bool*>(ptr.get());
    if (auto it = map.find(base + "IPAddress"); it != map.end() && it->second) {
      client_msg.ip_address = *static_cast<std::string*>(it->second.get());
    }
    if (auto it = map.find(base + "ProtocolVersion"); it != map.end() && it->second) {
      client_msg.protocol_version = static_cast<uint8_t>(*static_cast<int64_t*>(it->second.get()));
    }
    if (auto it = map.find(base + "RemotePort"); it != map.end() && it->second) {
      client_msg.remote_port = static_cast<uint16_t>(*static_cast<int64_t*>(it->second.get()));
    }

    misc_report_msg.nt_clients.push_back(std::move(client_msg));
  }

  misc_report_pub_->publish(misc_report_msg);
}

// void NTBridgeNode::on_string_topic_received(const std::shared_ptr<StringTopic::ValueType> msg) {
//   RCLCPP_INFO(this->get_logger(), "Yo! I received: %s", msg->c_str());
//   const auto& name_msg = string_topic_subscriber_->last_received_msg();
//   auto name_msg_ros = std_msgs::msg::String();
//   name_msg_ros.data = *name_msg;
//   my_name_pub_->publish(name_msg_ros);
// }

}  // namespace nt

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node =
    std::make_shared<nt::NTBridgeNode>("nt_bridge_node", rclcpp::NodeOptions());
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}