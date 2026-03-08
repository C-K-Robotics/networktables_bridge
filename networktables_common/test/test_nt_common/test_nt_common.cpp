// Copyright (c) 2025 C.K. Robotics

#include "test_nt_common.hpp"


namespace nt
{

TestNTCommonNode::TestNTCommonNode(const rclcpp::NodeOptions & options)
: Node("test_nt_common_node", options)
{
  // ROS2 Subscribers & Publishers
  data_1_copy_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/data/num_1", rclcpp::QoS(10), std::bind(&TestNTCommonNode::on_data_1_copy_received, this, std::placeholders::_1)
  );
  my_name_copy_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/data/my_name", rclcpp::QoS(10), std::bind(&TestNTCommonNode::on_my_name_copy_received, this, std::placeholders::_1)
  );
  data_1_pub_ = this->create_publisher<std_msgs::msg::Float64>("/data/num_1", rclcpp::QoS(10));
  data_2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/data/num_2", rclcpp::QoS(10));
  my_name_pub_ = this->create_publisher<std_msgs::msg::String>("/data/my_name", rclcpp::QoS(10));

  // RCLCPP Timers
  step_timer_20_hz_ =
    rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(0.05), [this] {
      step_20_hz();
    });
  step_timer_100_hz_ =
    rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(0.01), [this] {
      step_100_hz();
    });

  // NetworkTablesInstance Setup
  inst_ = nt::NetworkTableInstance::GetDefault();
  inst_.StartClient4("test_nt_common_client");
  inst_.SetServer("127.0.0.1");
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
  pubsub::subscribe_from<DoubleTopic>(inst_, double_topic_subscriber_1_, "/data/1");
  pubsub::subscribe_from<DoubleTopic>(inst_, double_topic_subscriber_2_, "/data/2", pubsub::kSensorPubSubOptions);
  pubsub::subscribe_from<StringTopic, TestNTCommonNode>(
    inst_, string_topic_subscriber_, "/data/my_name", this, &TestNTCommonNode::on_string_topic_received);
  pubsub::subscribe_from(inst_, multi_topic_subscriber_, {{"/data/array/"}});
  pubsub::publish_to<DoubleTopic>(inst_, double_topic_publisher_1_, "/data/1_copy");
  pubsub::publish_to<StringTopic>(inst_, string_topic_publisher_, "/data/my_name_copy");
  double_topic_publisher_1_->on_activate();
  string_topic_publisher_->on_activate();
}

void TestNTCommonNode::on_data_1_copy_received(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (msg == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Received null message on /data/num_1");
    return;
  }
  double_topic_publisher_1_->publish(std::make_unique<DoubleTopic::ValueType>(msg->data));
}

void TestNTCommonNode::on_my_name_copy_received(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Received null message on /data/my_name");
    return;
  }
  string_topic_publisher_->publish(msg->data);
}

void TestNTCommonNode::step_20_hz()
{
  // nt::SetNow(this->get_clock()->now().nanoseconds()); // Sync local NT time with ROS2 time
  // Publish some test data to ROS2 topics
  if (double_topic_subscriber_1_->has_msg())
  {
    const auto& msg1 = double_topic_subscriber_1_->last_received_msg();
    auto msg1_ros = std_msgs::msg::Float64();
    msg1_ros.data = *msg1;
    data_1_pub_->publish(msg1_ros);
  }

  if (double_topic_subscriber_2_->has_msg())
  {
    const auto& msg2 = double_topic_subscriber_2_->last_received_msg();
    auto msg2_ros = std_msgs::msg::Float64();
    msg2_ros.data = *msg2;
    data_2_pub_->publish(msg2_ros);
  }

  auto sp_ptr = std::static_pointer_cast<std::vector<int>>(
    multi_topic_subscriber_->last_received_msg("/data/array/bool_array"));
  auto sp_int_array = std::static_pointer_cast<std::vector<int64_t>>(
    multi_topic_subscriber_->last_received_msg("/data/array/int_array"));
  auto sp_double_array = std::static_pointer_cast<std::vector<double>>(
    multi_topic_subscriber_->last_received_msg("/data/array/double_array"));
  auto sp_string_array = std::static_pointer_cast<std::vector<std::string>>(
    multi_topic_subscriber_->last_received_msg("/data/array/string_array"));

  if (sp_ptr) PRINTOSS(sp_ptr);
  if (sp_int_array) PRINTOSS(sp_int_array);
  if (sp_double_array) PRINTOSS(sp_double_array);
  if (sp_string_array) PRINTOSS(sp_string_array);
}

void TestNTCommonNode::step_100_hz()
{
  auto msg1 = double_topic_subscriber_1_->last_received_msg();
  auto msg2 = double_topic_subscriber_2_->last_received_msg();
  auto msg3 = string_topic_subscriber_->last_received_msg();
  if (msg1 && msg2 && msg3)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received messages (1, 2, my name): (%f, %f, %s)\n"
      "Time Sync (ROS2 - NT): %ld",
      *msg1, *msg2, msg3->c_str(), this->get_clock()->now().nanoseconds() - static_cast<int64_t>(nt::Now()*1e3)
    );
  }
}

void TestNTCommonNode::on_string_topic_received(const std::shared_ptr<StringTopic::TimestampedValueType> msg) {
  RCLCPP_INFO(this->get_logger(), "Yo! I received: %s at time %ld", msg->value.c_str(), msg->time);
  const auto& name_msg = string_topic_subscriber_->last_received_msg();
  auto name_msg_ros = std_msgs::msg::String();
  name_msg_ros.data = *name_msg;
  my_name_pub_->publish(name_msg_ros);
}

}  // namespace nt

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nt::TestNTCommonNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}