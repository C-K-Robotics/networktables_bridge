// Copyright (c) 2025 C.K. Robotics

#include "networktables_common/test_nt_common.hpp"


namespace nt
{

TestNTCommonNode::TestNTCommonNode(const rclcpp::NodeOptions & options)
: Node("test_nt_common_node", options)
{
  // Declare parameters
  // this->declare_parameter<double>("gps_publish_s", 0.05);

  // dt_gps_ = this->get_parameter("gps_publish_s").as_double();
  // this->debug_ = this->get_parameter("debug").as_bool();

  // auto qos = rclcpp::QoS(10)
  //   .best_effort()
  //   .durability_volatile()
  //   .lifespan(std::chrono::nanoseconds::max())
  //   .deadline(std::chrono::nanoseconds::max())
  //   .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
  //   .liveliness_lease_duration(std::chrono::nanoseconds::max());

  // gps_subscriber_ = this->create_subscription<gps_msgs::msg::GPSFix>(
  //   "raw_gps", qos,
  //   std::bind(&GPSWaypointFollowerNode::gps_callback, this, std::placeholders::_1)
  // );

  // ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
  //   "ackermann_cmd", rclcpp::QoS(10),
  //   std::bind(&GPSWaypointFollowerNode::ackermann_callback, this, std::placeholders::_1)
  // );

  // ROS2 Publishers
  // odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gps_odom", rclcpp::QoS(10));
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

  // Subscribers via NetworkTables Common PubSub
  pubsub::subscribe_from<DoubleTopic>(this, inst_, double_topic_subscriber_1_, "/data/1");
  pubsub::subscribe_from<DoubleTopic>(this, inst_, double_topic_subscriber_2_, "/data/2");
  pubsub::subscribe_from<StringTopic, TestNTCommonNode>(
    this, inst_, string_topic_subscriber_, "/data/my name", &TestNTCommonNode::on_string_topic_received);
}

// void TestNTCommonNode::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
// {
//   // Implementation of the callback function
//   this->ackermann_cmd_ = *msg;
// }

void TestNTCommonNode::step_20_hz()
{
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
}

void TestNTCommonNode::step_100_hz()
{
  if (double_topic_subscriber_1_->has_msg() && double_topic_subscriber_2_->has_msg() && string_topic_subscriber_->has_msg()) {
    const auto& msg1 = double_topic_subscriber_1_->last_received_msg();
    const auto& msg2 = double_topic_subscriber_2_->last_received_msg();
    const auto& msg3 = string_topic_subscriber_->last_received_msg();
    RCLCPP_INFO(this->get_logger(), "Received messages (1, 2, my name): (%f, %f, %s)", *msg1, *msg2, msg3->c_str());
  }
}

void TestNTCommonNode::on_string_topic_received(const std::shared_ptr<StringTopic::ValueType> msg) {
  RCLCPP_INFO(this->get_logger(), "Yo! I received: %s", msg->c_str());
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