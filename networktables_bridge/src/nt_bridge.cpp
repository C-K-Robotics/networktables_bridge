// Copyright (c) 2025 C.K. Robotics

#include "networktables_bridge/nt_bridge.hpp"


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
  // pubsub::subscribe_from<DoubleTopic>(inst_, double_topic_subscriber_1_, "/data/1");
  // pubsub::subscribe_from<DoubleTopic>(inst_, double_topic_subscriber_2_, "/data/2", pubsub::kSensorPubSubOptions);
  // pubsub::subscribe_from<StringTopic, NTBridgeNode>(
  //   inst_, string_topic_subscriber_, "/data/my_name", this, &NTBridgeNode::on_string_topic_received);
  // double_topic_publisher_1_->on_activate();
  // string_topic_publisher_->on_activate();
  // RCLCPP_INFO(get_logger(), "on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// void NTBridgeNode::on_data_1_copy_received(const std_msgs::msg::Float64::SharedPtr msg)
// {
//   if (msg == nullptr) {
//     RCLCPP_WARN(this->get_logger(), "Received null message on /data/num_1");
//     return;
//   }
//   double_topic_publisher_1_->publish(std::make_unique<DoubleTopic::ValueType>(msg->data));
// }

// void NTBridgeNode::on_my_name_copy_received(const std_msgs::msg::String::SharedPtr msg)
// {
//   if (msg == nullptr) {
//     RCLCPP_WARN(this->get_logger(), "Received null message on /data/my_name");
//     return;
//   }
//   string_topic_publisher_->publish(msg->data);
// }

// void NTBridgeNode::step_20_hz()
// {
//   // nt::SetNow(this->get_clock()->now().nanoseconds()); // Sync local NT time with ROS2 time
//   // Publish some test data to ROS2 topics
//   if (double_topic_subscriber_1_->has_msg())
//   {
//     const auto& msg1 = double_topic_subscriber_1_->last_received_msg();
//     auto msg1_ros = std_msgs::msg::Float64();
//     msg1_ros.data = *msg1;
//     data_1_pub_->publish(msg1_ros);
//   }

//   if (double_topic_subscriber_2_->has_msg())
//   {
//     const auto& msg2 = double_topic_subscriber_2_->last_received_msg();
//     auto msg2_ros = std_msgs::msg::Float64();
//     msg2_ros.data = *msg2;
//     data_2_pub_->publish(msg2_ros);
//   }
// }

void NTBridgeNode::step_50_hz()
{
  // auto msg1 = double_topic_subscriber_1_->last_received_msg();
  // auto msg2 = double_topic_subscriber_2_->last_received_msg();
  // auto msg3 = string_topic_subscriber_->last_received_msg();
  // if (msg1 && msg2 && msg3)
  // {
  //   RCLCPP_INFO(
  //     this->get_logger(),
  //     "Received messages (1, 2, my name): (%f, %f, %s)\n"
  //     "Time Sync (ROS2 - NT): %ld",
  //     *msg1, *msg2, msg3->c_str(), this->get_clock()->now().nanoseconds() - static_cast<int64_t>(nt::Now()*1e3)
  //   );
  // }
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
  exe.add_node(
    std::make_shared<nt::NTBridgeNode>(
      "nt_bridge_node", rclcpp::NodeOptions())->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}