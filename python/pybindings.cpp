#include <memory>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "roboflex_core/core.h"
#include "roboflex_core/pybindings.h"
#include "roboflex_hiwonder_bus_servo/hiwonder_bus_servo.h"
#include "roboflex_hiwonder_bus_servo/hiwonder_bus_servo_controller.h"

namespace py = pybind11;

using namespace roboflex;
using namespace roboflex::core;
using namespace roboflex::hiwonderbusservo;
using namespace roboflex::hiwonderbusservonodes;

template <class NodeBase = HiwonderBusServoGroupControllerNode>
class PyHiwonderBusServoGroupControllerNode : public PyNode<NodeBase> {
public:
    using PyNode<NodeBase>::PyNode;

    HiwonderBusServoGroupCommand readwrite_loop_function(
        const HiwonderBusServoGroupState& state,
        const core::MessagePtr last_msg) override {
        PYBIND11_OVERRIDE_PURE(
            HiwonderBusServoGroupCommand,
            NodeBase,
            readwrite_loop_function,
            state,
            last_msg);
    }
};

template <class NodeBase = HiwonderBusServoRemoteController>
class PyHiwonderBusServoRemoteController : public PyNode<NodeBase> {
public:
    using PyNode<NodeBase>::PyNode;

    HiwonderBusServoGroupCommand readwrite_loop_function(
        const HiwonderBusServoGroupState& state) override {
        PYBIND11_OVERRIDE_PURE(
            HiwonderBusServoGroupCommand,
            NodeBase,
            readwrite_loop_function,
            state);
    }
};

template <class NodeBase = HiwonderBusServoRemoteFrequencyController>
class PyHiwonderBusServoRemoteFrequencyController : public PyNode<NodeBase> {
public:
    using PyNode<NodeBase>::PyNode;

    HiwonderBusServoGroupCommand readwrite_loop_function(
        const HiwonderBusServoGroupState& state) override {
        PYBIND11_OVERRIDE_PURE(
            HiwonderBusServoGroupCommand,
            NodeBase,
            readwrite_loop_function,
            state);
    }
};

PYBIND11_MODULE(roboflex_hiwonder_bus_servo_ext, m) {
    m.doc() = "roboflex_hiwonder_bus_servo_ext";

    py::register_exception<HiwonderBusServoException>(m, "HiwonderBusServoException");

    py::class_<TimestampPair>(m, "TimestampPair")
        .def(py::init<>())
        .def_readwrite("t0", &TimestampPair::t0)
        .def_readwrite("t1", &TimestampPair::t1);

    py::enum_<DynamicReadField>(m, "DynamicReadField")
        .value("Position", DynamicReadField::Position)
        .value("VoltageMV", DynamicReadField::VoltageMV)
        .value("TemperatureC", DynamicReadField::TemperatureC)
        .value("TorqueEnabled", DynamicReadField::TorqueEnabled);

    py::class_<HiwonderBusServoDynamicState>(m, "HiwonderBusServoDynamicState")
        .def(py::init<>())
        .def_readwrite("position", &HiwonderBusServoDynamicState::position)
        .def_readwrite("voltage_mv", &HiwonderBusServoDynamicState::voltage_mv)
        .def_readwrite("temperature_c", &HiwonderBusServoDynamicState::temperature_c)
        .def_readwrite("torque_enabled", &HiwonderBusServoDynamicState::torque_enabled)
        .def("__repr__", &HiwonderBusServoDynamicState::to_string);

    py::class_<HiwonderBusServoGroupState>(m, "HiwonderBusServoGroupState")
        .def(py::init<>())
        .def_readwrite("values", &HiwonderBusServoGroupState::values)
        .def_readwrite("timestamp", &HiwonderBusServoGroupState::timestamp)
        .def("__repr__", &HiwonderBusServoGroupState::to_string);

    py::class_<HiwonderBusServoGroupCommand>(m, "HiwonderBusServoGroupCommand")
        .def(py::init<>())
        .def_readwrite("positions", &HiwonderBusServoGroupCommand::positions)
        .def_readwrite("duration_ms", &HiwonderBusServoGroupCommand::duration_ms)
        .def_readwrite("should_write", &HiwonderBusServoGroupCommand::should_write)
        .def_readwrite("timestamp", &HiwonderBusServoGroupCommand::timestamp)
        .def("set", &HiwonderBusServoGroupCommand::set)
        .def("__repr__", &HiwonderBusServoGroupCommand::to_string);

    py::class_<DynamicReadConfig>(m, "DynamicReadConfig")
        .def(py::init<>())
        .def_readwrite("servo_ids", &DynamicReadConfig::servo_ids)
        .def_readwrite("every_n_loops", &DynamicReadConfig::every_n_loops)
        .def_readwrite("loop_sleep_ms", &DynamicReadConfig::loop_sleep_ms)
        .def("set_every_n_loops", &DynamicReadConfig::set_every_n_loops)
        .def("due_fields", &DynamicReadConfig::due_fields);

    py::class_<HiwonderBusServoController, std::shared_ptr<HiwonderBusServoController>>(m, "HiwonderBusServoController")
        .def(py::init<const std::string&, int, int, int>(),
            py::arg("device_name") = "/dev/rrc",
            py::arg("baud_rate") = 1000000,
            py::arg("timeout_ms") = 100,
            py::arg("retries") = 10)
        .def("get_device_name", &HiwonderBusServoController::get_device_name)
        .def("get_baud_rate", &HiwonderBusServoController::get_baud_rate)
        .def("get_timeout_ms", &HiwonderBusServoController::get_timeout_ms)
        .def("get_retries", &HiwonderBusServoController::get_retries)
        .def("write_positions", &HiwonderBusServoController::write_positions)
        .def("write_positions_ms", &HiwonderBusServoController::write_positions_ms)
        .def("stop", &HiwonderBusServoController::stop)
        .def("read_position", &HiwonderBusServoController::read_position)
        .def("read_voltage_mv", &HiwonderBusServoController::read_voltage_mv)
        .def("read_temperature_c", &HiwonderBusServoController::read_temperature_c)
        .def("read_torque_enabled", &HiwonderBusServoController::read_torque_enabled)
        .def("read_dynamic", &HiwonderBusServoController::read_dynamic)
        .def("read_id", &HiwonderBusServoController::read_id, py::arg("servo_id") = 254)
        .def("read_offset", &HiwonderBusServoController::read_offset)
        .def("read_angle_limit", &HiwonderBusServoController::read_angle_limit)
        .def("read_vin_limit", &HiwonderBusServoController::read_vin_limit)
        .def("read_temp_limit", &HiwonderBusServoController::read_temp_limit)
        .def("set_torque_enabled", &HiwonderBusServoController::set_torque_enabled)
        .def("set_id", &HiwonderBusServoController::set_id)
        .def("set_offset", &HiwonderBusServoController::set_offset)
        .def("save_offset", &HiwonderBusServoController::save_offset)
        .def("set_angle_limit", &HiwonderBusServoController::set_angle_limit)
        .def("set_vin_limit", &HiwonderBusServoController::set_vin_limit)
        .def("set_temp_limit", &HiwonderBusServoController::set_temp_limit)
        .def("set_loop_sleep_ms", &HiwonderBusServoController::set_loop_sleep_ms)
        .def("get_loop_sleep_ms", &HiwonderBusServoController::get_loop_sleep_ms)
        .def("run_readwrite_loop", [](
            HiwonderBusServoController& controller,
            const DynamicReadConfig& config,
            py::function rwf) {
            auto rwf_ptr = std::make_shared<py::function>(std::move(rwf));
            auto rwf_local = [rwf_ptr](
                const HiwonderBusServoGroupState& state,
                HiwonderBusServoGroupCommand& command) {
                py::gil_scoped_acquire gil_acquire;
                py::object result_obj = (*rwf_ptr)(state, &command);
                return result_obj.cast<bool>();
            };
            {
                py::gil_scoped_release release;
                controller.run_readwrite_loop(config, rwf_local);
            }
        }, py::arg("config"), py::arg("rwf"))
        .def("freeze", &HiwonderBusServoController::freeze);

    py::class_<HiwonderBusServoGroupStateMessage, core::Message, std::shared_ptr<HiwonderBusServoGroupStateMessage>>(m, "HiwonderBusServoGroupStateMessage")
        .def(py::init<const HiwonderBusServoGroupState&>())
        .def(py::init([](const std::shared_ptr<core::Message> other) {
            return std::make_shared<HiwonderBusServoGroupStateMessage>(*other);
        }))
        .def_property_readonly_static("MessageName", [](py::object) { return HiwonderBusServoGroupStateMessage::MessageName; })
        .def_property_readonly("state", &HiwonderBusServoGroupStateMessage::get_state)
        .def("__repr__", &HiwonderBusServoGroupStateMessage::to_string);

    py::class_<HiwonderBusServoGroupCommandMessage, core::Message, std::shared_ptr<HiwonderBusServoGroupCommandMessage>>(m, "HiwonderBusServoGroupCommandMessage")
        .def(py::init<const HiwonderBusServoGroupCommand&>())
        .def(py::init([](const std::shared_ptr<core::Message> other) {
            return std::make_shared<HiwonderBusServoGroupCommandMessage>(*other);
        }))
        .def_property_readonly_static("MessageName", [](py::object) { return HiwonderBusServoGroupCommandMessage::MessageName; })
        .def_property_readonly("command", &HiwonderBusServoGroupCommandMessage::get_command)
        .def("__repr__", &HiwonderBusServoGroupCommandMessage::to_string);

    py::class_<HiwonderBusServoStateCommandMessage, core::Message, std::shared_ptr<HiwonderBusServoStateCommandMessage>>(m, "HiwonderBusServoStateCommandMessage")
        .def(py::init<const HiwonderBusServoGroupState&, const HiwonderBusServoGroupCommand&>())
        .def(py::init([](const std::shared_ptr<core::Message> other) {
            return std::make_shared<HiwonderBusServoStateCommandMessage>(*other);
        }))
        .def_property_readonly_static("MessageName", [](py::object) { return HiwonderBusServoStateCommandMessage::MessageName; })
        .def_property_readonly("state", &HiwonderBusServoStateCommandMessage::get_state)
        .def_property_readonly("command", &HiwonderBusServoStateCommandMessage::get_command)
        .def("__repr__", &HiwonderBusServoStateCommandMessage::to_string);

    py::class_<HiwonderBusServoGroupControllerNode, core::RunnableNode, PyHiwonderBusServoGroupControllerNode<>, std::shared_ptr<HiwonderBusServoGroupControllerNode>>(m, "HiwonderBusServoGroupControllerNode")
        .def(py::init<std::shared_ptr<HiwonderBusServoController>, DynamicReadConfig, const std::string&>(),
            py::arg("controller"),
            py::arg("read_config"),
            py::arg("name") = "HBSGroupCtrl")
        .def_readonly("controller", &HiwonderBusServoGroupControllerNode::controller)
        .def_readonly("read_config", &HiwonderBusServoGroupControllerNode::read_config);

    py::class_<HiwonderBusServoGroupNode, core::RunnableNode, std::shared_ptr<HiwonderBusServoGroupNode>>(m, "HiwonderBusServoGroupNode")
        .def(py::init<std::shared_ptr<HiwonderBusServoController>, DynamicReadConfig, const std::string&>(),
            py::arg("controller"),
            py::arg("read_config"),
            py::arg("name") = "HBSGroupNode")
        .def_readonly("controller", &HiwonderBusServoGroupNode::controller)
        .def_readonly("read_config", &HiwonderBusServoGroupNode::read_config);

    py::class_<HiwonderBusServoRemoteController, core::Node, PyHiwonderBusServoRemoteController<>, std::shared_ptr<HiwonderBusServoRemoteController>>(m, "HiwonderBusServoRemoteController")
        .def(py::init<const std::string&>(),
            py::arg("name") = "HBSRemoteCtrl");

    py::class_<HiwonderBusServoRemoteFrequencyController, nodes::FrequencyGenerator, PyHiwonderBusServoRemoteFrequencyController<>, std::shared_ptr<HiwonderBusServoRemoteFrequencyController>>(m, "HiwonderBusServoRemoteFrequencyController")
        .def(py::init<float, const std::string&>(),
            py::arg("frequency_hz"),
            py::arg("name") = "HBSRemoteFreqCtrl");
}
