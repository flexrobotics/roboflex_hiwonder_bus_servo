#include <sstream>
#include "roboflex_hiwonder_bus_servo/hiwonder_bus_servo.h"

namespace roboflex {
namespace hiwonderbusservonodes {

namespace {

void write_group_state(
    flexbuffers::Builder& fbb,
    const HiwonderBusServoGroupState& state) {
    fbb.Map("state", [&]() {
        for (const auto& [servo_id, servo_state] : state.values) {
            auto servo_key = std::to_string(static_cast<int>(servo_id));
            fbb.Map(servo_key.c_str(), [&]() {
                if (servo_state.position.has_value()) {
                    fbb.Int("position", *servo_state.position);
                }
                if (servo_state.voltage_mv.has_value()) {
                    fbb.Int("voltage_mv", *servo_state.voltage_mv);
                }
                if (servo_state.temperature_c.has_value()) {
                    fbb.Int("temperature_c", *servo_state.temperature_c);
                }
                if (servo_state.torque_enabled.has_value()) {
                    fbb.Bool("torque_enabled", *servo_state.torque_enabled);
                }
            });
        }
    });
    fbb.Double("t0", state.timestamp.t0);
    fbb.Double("t1", state.timestamp.t1);
}

void write_group_command(
    flexbuffers::Builder& fbb,
    const HiwonderBusServoGroupCommand& command) {
    fbb.Map("command", [&]() {
        fbb.Int("duration_ms", command.duration_ms);
        fbb.Bool("should_write", command.should_write);
        fbb.Map("positions", [&]() {
            for (const auto& [servo_id, position] : command.positions) {
                auto servo_key = std::to_string(static_cast<int>(servo_id));
                fbb.Int(servo_key.c_str(), position);
            }
        });
        fbb.Double("t0", command.timestamp.t0);
        fbb.Double("t1", command.timestamp.t1);
    });
}

HiwonderBusServoGroupState read_group_state(const flexbuffers::Map root_map) {
    HiwonderBusServoGroupState state;
    auto state_map = root_map["state"].AsMap();
    auto keys = state_map.Keys();
    for (size_t i = 0; i < keys.size(); ++i) {
        auto key = keys[i].AsString().str();
        ServoId servo_id = static_cast<ServoId>(std::stoi(key));
        auto servo_map = state_map[key].AsMap();
        auto& servo_state = state.values[servo_id];

        auto position_ref = servo_map["position"];
        if (!position_ref.IsNull()) {
            servo_state.position = static_cast<int>(position_ref.AsInt32());
        }

        auto voltage_ref = servo_map["voltage_mv"];
        if (!voltage_ref.IsNull()) {
            servo_state.voltage_mv = static_cast<int>(voltage_ref.AsInt32());
        }

        auto temp_ref = servo_map["temperature_c"];
        if (!temp_ref.IsNull()) {
            servo_state.temperature_c = static_cast<int>(temp_ref.AsInt32());
        }

        auto torque_ref = servo_map["torque_enabled"];
        if (!torque_ref.IsNull()) {
            servo_state.torque_enabled = torque_ref.AsBool();
        }
    }

    state.timestamp.t0 = root_map["t0"].AsDouble();
    state.timestamp.t1 = root_map["t1"].AsDouble();
    return state;
}

HiwonderBusServoGroupCommand read_group_command(const flexbuffers::Reference ref) {
    HiwonderBusServoGroupCommand command;
    auto command_map = ref.AsMap();
    command.duration_ms = static_cast<int>(command_map["duration_ms"].AsInt32());
    command.should_write = command_map["should_write"].AsBool();

    auto positions = command_map["positions"].AsMap();
    auto keys = positions.Keys();
    for (size_t i = 0; i < keys.size(); ++i) {
        auto key = keys[i].AsString().str();
        ServoId servo_id = static_cast<ServoId>(std::stoi(key));
        command.positions[servo_id] = static_cast<int>(positions[key].AsInt32());
    }

    auto t0_ref = command_map["t0"];
    auto t1_ref = command_map["t1"];
    if (!t0_ref.IsNull()) {
        command.timestamp.t0 = t0_ref.AsDouble();
    }
    if (!t1_ref.IsNull()) {
        command.timestamp.t1 = t1_ref.AsDouble();
    }
    return command;
}

} // namespace

HiwonderBusServoGroupStateMessage::HiwonderBusServoGroupStateMessage(
    const HiwonderBusServoGroupState& state) :
        core::Message(ModuleName, MessageName) {
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        write_group_state(fbb, state);
    });
    state_ = get_state();
}

HiwonderBusServoGroupState HiwonderBusServoGroupStateMessage::get_state() const {
    if (!state_initialized_) {
        state_ = read_group_state(root_map());
        state_initialized_ = true;
    }
    return state_;
}

void HiwonderBusServoGroupStateMessage::print_on(std::ostream& os) const {
    os << "<HiwonderBusServoGroupStateMessage ";
    get_state().print_on(os);
    os << " ";
    core::Message::print_on(os);
    os << ">";
}

HiwonderBusServoGroupCommandMessage::HiwonderBusServoGroupCommandMessage(
    const HiwonderBusServoGroupCommand& command) :
        core::Message(ModuleName, MessageName) {
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        write_group_command(fbb, command);
    });
    command_ = get_command();
}

HiwonderBusServoGroupCommand HiwonderBusServoGroupCommandMessage::get_command() const {
    if (!command_initialized_) {
        command_ = read_group_command(root_val("command"));
        command_initialized_ = true;
    }
    return command_;
}

void HiwonderBusServoGroupCommandMessage::print_on(std::ostream& os) const {
    os << "<HiwonderBusServoGroupCommandMessage ";
    get_command().print_on(os);
    os << " ";
    core::Message::print_on(os);
    os << ">";
}

HiwonderBusServoStateCommandMessage::HiwonderBusServoStateCommandMessage(
    const HiwonderBusServoGroupState& state,
    const HiwonderBusServoGroupCommand& command) :
        core::Message(ModuleName, MessageName) {
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        write_group_state(fbb, state);
        write_group_command(fbb, command);
    });
    state_ = get_state();
    command_ = get_command();
}

HiwonderBusServoGroupState HiwonderBusServoStateCommandMessage::get_state() const {
    if (!state_initialized_) {
        state_ = read_group_state(root_map());
        state_initialized_ = true;
    }
    return state_;
}

HiwonderBusServoGroupCommand HiwonderBusServoStateCommandMessage::get_command() const {
    if (!command_initialized_) {
        command_ = read_group_command(root_val("command"));
        command_initialized_ = true;
    }
    return command_;
}

void HiwonderBusServoStateCommandMessage::print_on(std::ostream& os) const {
    os << "<HiwonderBusServoStateCommandMessage state:";
    get_state().print_on(os);
    os << " command:";
    get_command().print_on(os);
    os << " ";
    core::Message::print_on(os);
    os << ">";
}

HiwonderBusServoGroupControllerNode::HiwonderBusServoGroupControllerNode(
    HiwonderBusServoController::Ptr controller,
    DynamicReadConfig read_config,
    const std::string& name) :
        core::RunnableNode(name),
        controller(controller),
        read_config(read_config) {
}

void HiwonderBusServoGroupControllerNode::receive(core::MessagePtr m) {
    const std::lock_guard<std::recursive_mutex> lock(last_message_mutex);
    last_message = m;
}

void HiwonderBusServoGroupControllerNode::child_thread_fn() {
    auto fn = [this](const HiwonderBusServoGroupState& state, HiwonderBusServoGroupCommand& command) {
        bool should_continue = !this->stop_requested();
        if (should_continue) {
            const std::lock_guard<std::recursive_mutex> lock(last_message_mutex);
            command = this->readwrite_loop_function(state, last_message);
            this->signal(std::make_shared<HiwonderBusServoStateCommandMessage>(state, command));
        }
        return should_continue;
    };

    this->controller->run_readwrite_loop(read_config, fn);
    this->controller->freeze();
}

HiwonderBusServoGroupNode::HiwonderBusServoGroupNode(
    HiwonderBusServoController::Ptr controller,
    DynamicReadConfig read_config,
    const std::string& name) :
        core::RunnableNode(name),
        controller(controller),
        read_config(read_config) {
}

void HiwonderBusServoGroupNode::receive(core::MessagePtr m) {
    const std::lock_guard<std::recursive_mutex> lock(last_command_message_mutex);
    last_command_message = std::make_shared<HiwonderBusServoGroupCommandMessage>(*m);
}

bool HiwonderBusServoGroupNode::readwrite_loop_function(
    const HiwonderBusServoGroupState& state,
    HiwonderBusServoGroupCommand& command) {
    bool should_continue = !this->stop_requested();
    if (should_continue) {
        const std::lock_guard<std::recursive_mutex> lock(last_command_message_mutex);
        if (last_command_message == nullptr) {
            command.should_write = false;
        } else {
            command = last_command_message->get_command();
        }
        this->signal(std::make_shared<HiwonderBusServoGroupStateMessage>(state));
    }
    return should_continue;
}

void HiwonderBusServoGroupNode::child_thread_fn() {
    auto fn = [this](const HiwonderBusServoGroupState& state, HiwonderBusServoGroupCommand& command) {
        return this->readwrite_loop_function(state, command);
    };

    this->controller->run_readwrite_loop(read_config, fn);
    this->controller->freeze();
}

HiwonderBusServoRemoteController::HiwonderBusServoRemoteController(const std::string& name) :
    core::Node(name) {
}

void HiwonderBusServoRemoteController::receive(core::MessagePtr m) {
    if (m->message_name() == HiwonderBusServoGroupStateMessage::MessageName) {
        auto state = HiwonderBusServoGroupStateMessage(*m).get_state();
        auto command = this->readwrite_loop_function(state);
        this->signal(std::make_shared<HiwonderBusServoGroupCommandMessage>(command));
    }
}

HiwonderBusServoRemoteFrequencyController::HiwonderBusServoRemoteFrequencyController(
    float frequency_hz,
    const std::string& name) :
        nodes::FrequencyGenerator(frequency_hz, name) {
}

void HiwonderBusServoRemoteFrequencyController::receive(core::MessagePtr m) {
    if (m->message_name() == HiwonderBusServoGroupStateMessage::MessageName) {
        state_ = HiwonderBusServoGroupStateMessage(*m).get_state();
    }
}

void HiwonderBusServoRemoteFrequencyController::on_trigger(double wall_clock_time) {
    (void) wall_clock_time;
    if (state_.timestamp.t0 != 0.0 || state_.timestamp.t1 != 0.0) {
        auto command = this->readwrite_loop_function(state_);
        this->signal(std::make_shared<HiwonderBusServoGroupCommandMessage>(command));
    }
}

} // hiwonderbusservonodes
} // roboflex
