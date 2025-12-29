# Copyright (c) 2025 C.K. Robotics

import ntcore as nt
from typing import Any


kDefaultPubSubOptions = nt.PubSubOptions()
kSensorPubSubOptions = nt.PubSubOptions(periodic=0.02)  # 50 Hz

TopicValueType: dict[type, Any] = {
  nt.BooleanArrayTopic: list[bool],
  nt.BooleanTopic: bool,
  nt.DoubleArrayTopic: list[float],
  nt.DoubleTopic: float,
  nt.FloatArrayTopic: list[float],
  nt.FloatTopic: float,
  nt.IntegerArrayTopic: list[int],
  nt.IntegerTopic: int,
  nt.RawTopic: bytearray,
  nt.StringArrayTopic: list[str],
  nt.StringTopic: str,
  # no protobuf topic
  nt.StructArrayTopic: list[bytearray],
  nt.StructTopic: bytearray,
}

# TODO(Winston): consider restricting TopicT and msg types
class TopicPublisher:
  def __init__(self, TopicT,
               inst: nt.NetworkTableInstance,
               topic_name: str,
               options: nt.PubSubOptions = kDefaultPubSubOptions):
    self.__inst = inst
    self.__latest_msg_time = nt._now()

    name = topic_name[(topic_name.rfind("/")) + 1:]
    table_name = topic_name[:(topic_name.rfind("/"))]

    self.__publisher = TopicT(self.__inst.getTable(table_name).getTopic(name)).publish(options)
    self.__activated = False

  def publish(self, msg):
    self.__publisher.set(msg)
    self.__latest_msg_time = nt._now()

  def latest_msg_time(self) -> int:
    return self.__latest_msg_time

  def on_activate(self) -> bool:
    self.__activated = True
    return True

  def on_deactivate(self) -> bool:
    self.__activated = False
    return True

  def is_activated(self) -> bool:
    return self.__activated

class TopicSubscriber:
  def __init__(self, TopicT,
               inst: nt.NetworkTableInstance,
               topic_name: str,
               callback = None,
               options: nt.PubSubOptions = kDefaultPubSubOptions,
               default_msg = None):
    self.__inst = inst
    self.__latest_msg_time = nt._now()
    self.__callback = callback
    self.__last_received_msg = None
    self.__has_seen_msg = False

    name = topic_name[(topic_name.rfind("/")) + 1:]
    table_name = topic_name[:(topic_name.rfind("/"))]

    if default_msg is None:
      default_msg = TopicValueType[TopicT]()
    elif type(default_msg) == TopicValueType[TopicT]:
      pass
    else:
      raise TypeError("default_msg type does not match TopicT value type"
                      "while creating TopicSubscriber")

    self.__subscription = TopicT(self.__inst.getTable(table_name).getTopic(name)).subscribe(default_msg, options)

    self.__value_listener_handle = self.__inst.addListener(
      self.__subscription, nt.EventFlags.kValueAll, self.__event_callback
    )

  def __event_callback(self, event: nt.Event):
    msg = self.__subscription.get()
    self.__on_msg_received(msg)
    if self.__callback is not None:
      self.__callback(msg)

  def __on_msg_received(self, msg):
    self.__has_seen_msg = True
    self.__last_received_msg = msg
    self.__latest_msg_time = nt._now()

  def __del__(self):
    self.__inst.removeListener(self.__value_listener_handle)

  def take(self):
    msg = self.__last_received_msg.copy()
    self.__last_received_msg = None
    return msg

  def has_seen_msg(self) -> bool:
    return self.__has_seen_msg

  def has_msg(self) -> bool:
    return self.__last_received_msg is not None

  def latest_msg_time(self) -> int:
    return self.__latest_msg_time

  def last_received_msg(self):
    return self.__last_received_msg

def activate_publisher(publisher: TopicPublisher):
  publisher.on_activate()

def deactivate_publisher(publisher: TopicPublisher):
  publisher.on_deactivate()
