## Using networktables_common

This repository includes `networktables_common`, a small wrapper that exposes a publisher/subscriber-style API around WPILib/ntcore topics. It provides:

- `TopicPublisher<TopicT>` — publish NT topic values (templated on NT topic type, e.g. `nt::DoubleTopic`).
- `TopicSubscriber<TopicT>` — subscribe to a single NT topic and fetch the latest value.
- `MultiTopicSubscriber` — subscribe to many topics using a prefix list and query by topic name.
- Convenience helpers: `publish_to`, `subscribe_from`, `activate_publisher`, `deactivate_publisher`.

Simple C++ example (subscribe to AdvantageKit/SystemStats and publish a single double):

```cpp
#include <networktables/NetworkTableInstance.h>
#include "networktables_common/nt_pubsub.hpp"

// create/get NT instance and connect
auto inst = nt::NetworkTableInstance::GetDefault();
inst.StartClient4("my_ros_client");
inst.SetServer("10.80.20.2");

// Multi-topic subscriber for AdvantageKit/SystemStats prefix
std::shared_ptr<nt::pubsub::MultiTopicSubscriber> sys_stats_sub;
nt::pubsub::subscribe_from(inst, sys_stats_sub, {{"/AdvantageKit/SystemStats/"}});

// Publisher example (battery voltage)
std::shared_ptr<nt::pubsub::TopicPublisher<nt::DoubleTopic>> battery_pub;
nt::pubsub::publish_to<nt::DoubleTopic>(inst, battery_pub, "/Sensors/BatteryVoltage", /*activate=*/true);

// in your loop / timer
if (sys_stats_sub) {
	auto last = sys_stats_sub->last_received_msg("/AdvantageKit/SystemStats/BatteryVoltage");
	if (last) {
		double bv = *std::static_pointer_cast<double>(last);
		// republish or use in ROS message
		if (battery_pub && battery_pub->is_activated()) {
			battery_pub->publish(bv);
		}
	}
}
```

Simple Python example (same idea):

```python
from networktables_common import nt_pubsub
import ntcore as nt

inst = nt.NetworkTableInstance.getDefault()
inst.startClient4('py_client')
inst.setServer('10.80.20.2')

# subscribe
sub = nt_pubsub.TopicSubscriber(nt.DoubleTopic, inst, '/AdvantageKit/SystemStats/BatteryVoltage')

# publish
pub = nt_pubsub.TopicPublisher(nt.DoubleTopic, inst, '/Sensors/BatteryVoltage')
pub.on_activate()
pub.publish(12.34)

# check for new values
if sub.has_msg():
		val = sub.last_received_msg()
		print('Battery voltage:', val)
```

Notes:
- `MultiTopicSubscriber` returns generic typed values (stored as `std::shared_ptr<void>` in C++); use the topic name to look up the value and cast to the expected type.
- Publishers and subscribers accept `PubSubOptions` (e.g., `kSensorPubSubOptions`) to set periodic sampling.
- The C++ helpers are in `networktables_common/include/networktables_common/nt_pubsub.hpp` and a thin Python counterpart is available at `networktables_common/networktables_common/nt_pubsub.py`.

