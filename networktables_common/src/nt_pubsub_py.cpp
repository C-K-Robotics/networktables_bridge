// Copyright (c) 2025 C.K. Robotics
// TODO(Winston): Consider removing pybind11 bindings if not needed.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <memory>
#include <string>

#include "networktables_common/nt_pubsub.hpp"

namespace py = pybind11;

using nt::pubsub::TopicPublisher;
using nt::pubsub::TopicSubscriber;

PYBIND11_MODULE(nt_pubsub, m) {
  // Ensure RobotPy's bindings are imported so the type is registered in pybind11
  py::module_::import("ntcore._ntcore");  // important for cross-module cast

  // Bind only concrete instantiations (no templates exposed to Python)
  using DoubleTopicPublisher = TopicPublisher<nt::DoubleTopic>;
  using DoubleTopicSubscriber = TopicSubscriber<nt::DoubleTopic>;

  m.def("probe_instance", [](py::object inst_obj) {
    py::module_::import("ntcore._ntcore");
    auto &inst = inst_obj.cast<nt::NetworkTableInstance&>();
    return inst.IsConnected();
  });

   py::class_<DoubleTopicPublisher, std::shared_ptr<DoubleTopicPublisher>>(m, "DoubleTopicPublisher")
    .def(py::init([](py::object inst_obj,
                     const std::string& topic,
                     std::optional<nt::PubSubOptions> options_opt) {
        auto &inst = inst_obj.cast<nt::NetworkTableInstance&>();
        const auto &opts = options_opt ? *options_opt : nt::kDefaultPubSubOptions;
        return std::make_shared<DoubleTopicPublisher>(inst, topic, opts);
    }),
    py::arg("instance"), py::arg("topic"), py::arg("options") = py::none())
    .def("publish", py::overload_cast<const double&>(&DoubleTopicPublisher::publish))
    .def("activate", &DoubleTopicPublisher::on_activate)
    .def("deactivate", &DoubleTopicPublisher::on_deactivate)
    .def("is_activated", &DoubleTopicPublisher::is_activated)
    .def_property_readonly("latest_msg_time", &DoubleTopicPublisher::latest_msg_time);

  py::class_<DoubleTopicSubscriber, std::shared_ptr<DoubleTopicSubscriber>>(m, "DoubleTopicSubscriber")
    .def(py::init([](py::object inst_obj,
                     const std::string& topic,
                     std::optional<nt::PubSubOptions> options_opt,
                     double default_value) {
        auto &inst = inst_obj.cast<nt::NetworkTableInstance&>();
        const auto &opts = options_opt ? *options_opt : nt::kDefaultPubSubOptions;
        return std::make_shared<DoubleTopicSubscriber>(inst, topic, opts, default_value);
    }),
    py::arg("instance"), py::arg("topic"),
    py::arg("options") = py::none(),
    py::arg("default") = 0.0)
    .def("has_msg", &DoubleTopicSubscriber::has_msg)
    .def("has_seen_msg", &DoubleTopicSubscriber::has_seen_msg)
    .def("last", [](const DoubleTopicSubscriber& self) -> std::optional<double> {
        auto p = self.last_received_msg();
        if (!p) return std::nullopt;
        return *p;
    })
    .def("take", [](DoubleTopicSubscriber& self) -> std::optional<double> {
        auto p = self.take();
        if (!p) return std::nullopt;
        return *p;
    })
    .def_property_readonly("latest_msg_time", &DoubleTopicSubscriber::latest_msg_time);

  m.def("debug_type", [](py::object o) {
    return py::str(o.get_type());
  });
}