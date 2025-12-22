// Copyright (c) 2025 C.K. Robotics

#ifndef NETWORKTABLES_COMMON__NT_PUBSUB_HPP_
#define NETWORKTABLES_COMMON__NT_PUBSUB_HPP_

#include <memory>
#include <string>

#include <ntcore.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/BooleanArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/FloatTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/RawTopic.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/ProtobufTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace nt
{

namespace pubsub
{

// Declare non-templated functions
void split_topic_name(
  const std::string& topic_name,
  std::string& table_name,
  std::string& name);

template<typename TopicT, typename NodeT = rclcpp::Node>
class TopicSubscriber
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TopicSubscriber<TopicT, NodeT>)

  TopicSubscriber(
    NodeT * const parent,
    nt::NetworkTableInstance & inst,
    std::string const & topic_name,
    PubSubOptions const & options = kDefaultPubSubOptions,
    typename TopicT::ValueType const & default_msg = typename TopicT::ValueType{})
  {
    parent_ = parent;
    inst_ = inst;
    latest_msg_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);

    std::string name;
    std::string table_name;
    split_topic_name(topic_name, table_name, name);

    // TODO(Winston): Make sure the subscription_ outlives longer than the TopicT
    subscription_ = TopicT{inst_.GetTable(table_name)->GetTopic(name)}.Subscribe(default_msg, options);

    value_listener_handle_ = inst_.AddListener(
      subscription_,
      nt::EventFlags::kValueAll,
      [this] (const nt::Event& event) {
        std::scoped_lock lock{mutex_};
        const auto& msg = subscription_.Get();
        on_msg_received(std::make_shared<typename TopicT::ValueType>(msg));
      }
    );
  }

  ~TopicSubscriber() {
    inst_ = nt::NetworkTableInstance::GetDefault();
    inst_.RemoveListener(value_listener_handle_);
  }

  std::shared_ptr<typename TopicT::ValueType> take()
  {
    auto msg = last_received_msg_;
    last_received_msg_ = nullptr;
    return msg;
  }

  bool has_seen_msg()
  {
    return has_seen_msg_;
  }

  bool has_msg()
  {
    return last_received_msg_ != nullptr;
  }

  rclcpp::Time latest_msg_time()
  {
    return latest_msg_time_;
  }

  [[nodiscard]] std::shared_ptr<typename TopicT::ValueType> last_received_msg() const
  {
    return last_received_msg_;
  }

private:
  bool has_seen_msg_{};
  rclcpp::Time latest_msg_time_;
  NodeT * parent_;

  nt::NetworkTableInstance inst_;
  std::shared_ptr<typename TopicT::ValueType> last_received_msg_;
  typename TopicT::SubscriberType subscription_;

  std::mutex mutex_;  // use a mutex to make updating the value and flag thread-safe
  NT_Listener value_listener_handle_;

  void on_msg_received(const std::shared_ptr<typename TopicT::ValueType> msg)
  {
    has_seen_msg_ = true;
    last_received_msg_ = msg;
    latest_msg_time_ = parent_->now();
  }

  // TODO(Winston): Implement default message generation if needed
  // typename TopicT::ValueType get_default_msg()
  // {
  //   return typename TopicT::ValueType{};
  // }
};

// template<class TopicT>
// void publish_to(
//   rclcpp::Node * this_ptr,
//   typename std::shared_ptr<rclcpp::Publisher<TopicT>> & publisher,
//   const std::string & topic_name,
//   const PubSubOptions & options = kDefaultPubSubOptions)
// {
//   publisher = this_ptr->create_publisher<TopicT>(topic_name, options);
// }

// template<class TopicT>
// void publish_to(
//   rclcpp_lifecycle::LifecycleNode * this_ptr,
//   typename std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<TopicT>> & publisher,
//   const std::string & topic_name,
//   const PubSubOptions & options = kDefaultPubSubOptions)
// {
//   publisher = this_ptr->create_publisher<TopicT>(topic_name, options);
// }

// template<class TopicT>
// void activate_publisher(
//   typename std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<TopicT>> & publisher)
// {
//   publisher->on_activate();
// }

// template<class TopicT>
// void deactivate_publisher(
//   typename std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<TopicT>> & publisher)
// {
//   publisher->on_deactivate();
// }

// template<class TopicT, class NodeT>
// void subscribe_from(
//   NodeT * this_ptr,
//   typename std::shared_ptr<rclcpp::Subscription<TopicT>> & subscriber,
//   const std::string & topic_name,
//   void (NodeT::* callback)(typename std::shared_ptr<TopicT>),
//   const PubSubOptions & options = kDefaultPubSubOptions)
// {
//   subscriber =
//     static_cast<rclcpp::Node *>(this_ptr)->create_subscription<TopicT>(
//     topic_name, options,
//     std::bind(callback, this_ptr, std::placeholders::_1));
// }

template<class TopicT>
void subscribe_from(
  rclcpp::Node * const parent,
  nt::NetworkTableInstance & inst,
  typename std::unique_ptr<TopicSubscriber<TopicT>> & subscriber,
  const std::string & topic_name,
  const PubSubOptions & options = kDefaultPubSubOptions)
{
  subscriber = std::make_unique<TopicSubscriber<TopicT>>(parent, inst, topic_name, options);
}

template<class TopicT>
void subscribe_from(
  rclcpp_lifecycle::LifecycleNode * const parent,
  nt::NetworkTableInstance & inst,
  typename std::unique_ptr<TopicSubscriber<TopicT, rclcpp_lifecycle::LifecycleNode>> & subscriber,
  const std::string & topic_name,
  const PubSubOptions & options = kDefaultPubSubOptions)
{
  subscriber = std::make_unique<TopicSubscriber<TopicT, rclcpp_lifecycle::LifecycleNode>>(
    parent,
    topic_name,
    options
  );
}

template<class TopicT>
void subscribe_from(
  rclcpp::Node * const parent,
  nt::NetworkTableInstance & inst,
  typename std::shared_ptr<TopicSubscriber<TopicT>> & subscriber,
  const std::string & topic_name,
  const PubSubOptions & options = kDefaultPubSubOptions)
{
  subscriber = std::make_shared<TopicSubscriber<TopicT>>(parent, topic_name, options);
}

void split_topic_name(
  const std::string& topic_name,
  std::string& table_name,
  std::string& name)
{
  std::size_t pos = topic_name.rfind('/');
  if (pos == std::string::npos) {
    // no '/' found: whole string is the "name", table is empty
    table_name.clear();
    name = topic_name;
    return;
  }
  table_name = topic_name.substr(0, pos); // before last '/'
  name = topic_name.substr(pos + 1);      // after last '/'
}

}  // namespace pubsub

}  // namespace nt

#endif  // BASE_COMMON__PUBSUB_HPP_
