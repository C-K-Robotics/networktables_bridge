// Copyright (c) 2025 C.K. Robotics

#ifndef NETWORKTABLES_COMMON__NT_PUBSUB_HPP_
#define NETWORKTABLES_COMMON__NT_PUBSUB_HPP_

#include <memory>
#include <string>

#include <ntcore.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/MultiSubscriber.h>
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

namespace nt
{

namespace pubsub
{

constexpr PubSubOptions kSensorPubSubOptions = PubSubOptions{
  .periodic = 0.02,  // 50 Hz
};

// Declare non-templated functions
void split_topic_name(
  const std::string& topic_name,
  std::string& table_name,
  std::string& name);

template<typename TopicT>
class TopicPublisher
{
public:
  TopicPublisher(
    nt::NetworkTableInstance & inst,
    std::string const & topic_name,
    PubSubOptions const & options = kDefaultPubSubOptions)
  {
    inst_ = inst;
    latest_msg_time_ = nt::Now();

    std::string name;
    std::string table_name;
    split_topic_name(topic_name, table_name, name);
    publisher_ = TopicT{inst_.GetTable(table_name)->GetTopic(name)}.Publish(options);
  }

  void publish(std::unique_ptr<typename TopicT::ValueType> msg)
  {
    if (!is_activated() || !msg) return;
    publish(*msg);
  }

  void publish(const typename TopicT::ValueType & msg)
  {
    if (!is_activated()) return;
    publisher_.Set(msg);
    latest_msg_time_ = nt::Now();
  }

  int64_t latest_msg_time()
  {
    return latest_msg_time_;
  }

  void on_activate()
  {
    activated_.store(true);
  }

  void on_deactivate()
  {
    activated_.store(false);
  }

  bool is_activated() const
  {
    return activated_.load();
  }

private:
  std::atomic<bool> activated_ = false;
  int64_t latest_msg_time_;

  nt::NetworkTableInstance inst_;
  typename TopicT::PublisherType publisher_;
};

template<typename TopicT>
class TopicSubscriber
{
public:
  TopicSubscriber(
    nt::NetworkTableInstance & inst,
    std::string const & topic_name,
    PubSubOptions const & options = kDefaultPubSubOptions,
    typename TopicT::ValueType const & default_msg = typename TopicT::ValueType{})
  {
    inst_ = inst;
    latest_msg_time_ = nt::Now();

    std::string name;
    std::string table_name;
    split_topic_name(topic_name, table_name, name);
    subscription_ = TopicT{inst_.GetTable(table_name)->GetTopic(name)}.Subscribe(default_msg, options);

    value_listener_handle_ = inst_.AddListener(
      subscription_,
      nt::EventFlags::kValueAll,
      [this] (const nt::Event&) {
        auto msg = std::make_shared<typename TopicT::ValueType>(subscription_.Get());
        on_msg_received(msg);
      }
    );
  }

  template<typename ClassT>
  TopicSubscriber(
    nt::NetworkTableInstance & inst,
    std::string const & topic_name,
    ClassT * const this_ptr,
    void (ClassT::* callback)(const std::shared_ptr<typename TopicT::ValueType>),
    PubSubOptions const & options = kDefaultPubSubOptions,
    typename TopicT::ValueType const & default_msg = typename TopicT::ValueType{})
    : TopicSubscriber(inst, topic_name, options, default_msg)
  {
    inst_.RemoveListener(value_listener_handle_);
    value_listener_handle_ = inst_.AddListener(
      subscription_,
      nt::EventFlags::kValueAll,
      [this, this_ptr, callback] (const nt::Event&) {
        auto msg = std::make_shared<typename TopicT::ValueType>(subscription_.Get());
        on_msg_received(msg);
        (this_ptr->*callback)(msg);
      }
    );
  }

  ~TopicSubscriber()
  {
    inst_.RemoveListener(value_listener_handle_);
  }

  std::shared_ptr<typename TopicT::ValueType> take()
  {
    std::scoped_lock lock{mutex_};
    auto msg = last_received_msg_;
    last_received_msg_ = nullptr;
    return msg;
  }

  bool has_seen_msg()
  {
    std::scoped_lock lock{mutex_};
    return has_seen_msg_;
  }

  bool has_msg()
  {
    std::scoped_lock lock{mutex_};
    return last_received_msg_ != nullptr;
  }

  int64_t latest_msg_time()
  {
    std::scoped_lock lock{mutex_};
    return latest_msg_time_;
  }

  [[nodiscard]] std::shared_ptr<typename TopicT::ValueType> last_received_msg() const
  {
    std::scoped_lock lock{mutex_};
    return last_received_msg_;
  }

private:
  bool has_seen_msg_{};
  int64_t latest_msg_time_;

  nt::NetworkTableInstance inst_;
  std::shared_ptr<typename TopicT::ValueType> last_received_msg_;
  typename TopicT::SubscriberType subscription_;

  mutable std::mutex mutex_;  // use a mutex to make updating the value and flag thread-safe
  NT_Listener value_listener_handle_;

  void on_msg_received(const std::shared_ptr<typename TopicT::ValueType> msg)
  {
    std::scoped_lock lock{mutex_};
    has_seen_msg_ = true;
    last_received_msg_ = msg;
    latest_msg_time_ = nt::Now();
  }
};

class MultiTopicSubscriber
{
public:
  MultiTopicSubscriber(
    nt::NetworkTableInstance & inst,
    std::span<const std::string_view> const & prefixes,
    PubSubOptions const & options = kDefaultPubSubOptions)
  {
    inst_ = inst;
    subscription_ = nt::MultiSubscriber{inst_, prefixes, options};
    value_listener_handle_ = inst_.AddListener(
      subscription_,
      nt::EventFlags::kValueAll,
      [this] (const nt::Event& event) {
        nt::NetworkTableValue nt_value = event.GetValueEventData()->value;
        if (!nt_value.IsValid()) { return; }
        std::string topic_name = nt::GetTopicName(event.GetValueEventData()->topic);
        on_value_received(nt_value, topic_name);
      }
    );
  }

  template<typename ClassT>
  MultiTopicSubscriber(
    nt::NetworkTableInstance & inst,
    std::span<const std::string_view> const & prefixes,
    ClassT * const this_ptr,
    void (ClassT::* callback)(const nt::NetworkTableValue & value),
    PubSubOptions const & options = kDefaultPubSubOptions)
    : MultiTopicSubscriber(inst, prefixes, options)
  {
    inst_.RemoveListener(value_listener_handle_);
    value_listener_handle_ = inst_.AddListener(
      subscription_,
      nt::EventFlags::kValueAll,
      [this, this_ptr, callback] (const nt::Event& event) {
        nt::NetworkTableValue nt_value = event.GetValueEventData()->value;
        if (!nt_value.IsValid()) { return; }
        std::string & topic_name = nt::GetTopicName(event.GetValueEventData()->topic);
        on_value_received(nt_value, topic_name);
        (this_ptr->*callback)(nt_value);
      }
    );
  }

  ~MultiTopicSubscriber()
  {
    inst_.RemoveListener(value_listener_handle_);
  }

  int64_t latest_msg_time(const std::string & topic_name)
  {
    std::scoped_lock lock{mutex_};
    auto it = latest_msg_time_s_.find(topic_name);
    return (it == latest_msg_time_s_.end() || !it->second) ? -1 : it->second;
  }

  void* last_received_msg(const std::string & topic_name) const
  {
    std::scoped_lock lock{mutex_};
    auto it = last_received_msgs_.find(topic_name);
    return (it == last_received_msgs_.end() || !it->second) ? nullptr : it->second.get();
  }

  const std::unordered_map<std::string, std::shared_ptr<void>> &
  last_received_msgs()
  {
    std::scoped_lock lock{mutex_};
    return last_received_msgs_;
  }

private:
  std::unordered_map<std::string, int64_t> latest_msg_time_s_;
  std::unordered_map<std::string, std::shared_ptr<void>> last_received_msgs_;
  nt::NetworkTableInstance inst_;
  nt::MultiSubscriber subscription_;

  mutable std::mutex mutex_;  // use a mutex to make updating the value and flag thread-safe
  NT_Listener value_listener_handle_;

  void on_value_received(const nt::NetworkTableValue & value, const std::string & topic_name)
  {
    std::shared_ptr<void> msg;
    // TODO(Winston): std::span does not own memory, so this may lead to dangling pointers.
    if (value.IsBooleanArray()) {
      msg = std::make_shared<std::span<const int>>(value.GetBooleanArray());
    } else if (value.IsBoolean()) {
      msg = std::make_shared<bool>(value.GetBoolean());
    } else if (value.IsDoubleArray()) {
      msg = std::make_shared<std::span<const double>>(value.GetDoubleArray());
    } else if (value.IsDouble()) {
      msg = std::make_shared<double>(value.GetDouble());
    } else if (value.IsFloatArray()) {
      msg = std::make_shared<std::span<const float>>(value.GetFloatArray());
    } else if (value.IsFloat()) {
      msg = std::make_shared<float>(value.GetFloat());
    } else if (value.IsIntegerArray()) {
      msg = std::make_shared<std::span<const int64_t>>(value.GetIntegerArray());
    } else if (value.IsInteger()) {
      msg = std::make_shared<int64_t>(value.GetInteger());
    } else if (value.IsRaw()) {
      msg = std::make_shared<std::span<const uint8_t>>(value.GetRaw());
    } else if (value.IsStringArray()) {
      msg = std::make_shared<std::span<const std::string>>(value.GetStringArray());
    } else if (value.IsString()) {
      msg = std::make_shared<std::string>(value.GetString());
    } else {
      // Unsupported type
      return;
    }

    std::scoped_lock lock{mutex_};
    last_received_msgs_[topic_name] = msg;
    latest_msg_time_s_[topic_name] = nt::Now();
  }
};

template<class TopicT>
void publish_to(
  nt::NetworkTableInstance & inst,
  typename std::shared_ptr<TopicPublisher<TopicT>> & publisher,
  const std::string & topic_name,
  const bool & activate = false,
  const PubSubOptions & options = kDefaultPubSubOptions)
{
  publisher = std::make_shared<TopicPublisher<TopicT>>(inst, topic_name, options);
  if (activate) {
    publisher->on_activate();
  }
}

template<class TopicT>
void activate_publisher(
  typename std::shared_ptr<TopicPublisher<TopicT>> & publisher)
{
  publisher->on_activate();
}

template<class TopicT>
void deactivate_publisher(
  typename std::shared_ptr<TopicPublisher<TopicT>> & publisher)
{
  publisher->on_deactivate();
}

template<class TopicT>
void subscribe_from(
  nt::NetworkTableInstance & inst,
  typename std::unique_ptr<TopicSubscriber<TopicT>> & subscriber,
  const std::string & topic_name,
  const PubSubOptions & options = kDefaultPubSubOptions,
  typename TopicT::ValueType const & default_msg = typename TopicT::ValueType{})
{
  subscriber = std::make_unique<TopicSubscriber<TopicT>>(
    inst, topic_name, options, default_msg
  );
}

template<class TopicT>
void subscribe_from(
  nt::NetworkTableInstance & inst,
  typename std::shared_ptr<TopicSubscriber<TopicT>> & subscriber,
  const std::string & topic_name,
  const PubSubOptions & options = kDefaultPubSubOptions,
  typename TopicT::ValueType const & default_msg = typename TopicT::ValueType{})
{
  subscriber = std::make_shared<TopicSubscriber<TopicT>>(
    inst, topic_name, options, default_msg
  );
}

template<class TopicT, class ClassT>
void subscribe_from(
  nt::NetworkTableInstance & inst,
  typename std::shared_ptr<TopicSubscriber<TopicT>> & subscriber,
  const std::string & topic_name,
  ClassT * this_ptr,
  void (ClassT::* callback)(const std::shared_ptr<typename TopicT::ValueType>),
  const PubSubOptions & options = kDefaultPubSubOptions,
  typename TopicT::ValueType const & default_msg = typename TopicT::ValueType{})
{
  subscriber = std::make_shared<TopicSubscriber<TopicT>>(
    inst, topic_name, this_ptr, callback, options, default_msg
  );
}

void subscribe_from(
  nt::NetworkTableInstance & inst,
  std::shared_ptr<MultiTopicSubscriber> & subscriber,
  const std::span<const std::string_view> & prefixes,
  const PubSubOptions & options = kDefaultPubSubOptions)
{
  subscriber = std::make_shared<MultiTopicSubscriber>(
    inst, prefixes, options
  );
}

template<class ClassT>
void subscribe_from(
  nt::NetworkTableInstance & inst,
  std::shared_ptr<MultiTopicSubscriber> & subscriber,
  const std::span<const std::string_view> & prefixes,
  ClassT * this_ptr,
  void (ClassT::* callback)(const nt::NetworkTableValue & value),
  const PubSubOptions & options = kDefaultPubSubOptions)
{
  subscriber = std::make_shared<MultiTopicSubscriber>(
    inst, prefixes, this_ptr, callback, options
  );
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

#endif  // NETWORKTABLES_COMMON__NT_PUBSUB_HPP_
