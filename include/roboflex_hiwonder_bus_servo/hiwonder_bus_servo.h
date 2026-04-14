#ifndef ROBOFLEX_HIWONDER_BUS_SERVO__H
#define ROBOFLEX_HIWONDER_BUS_SERVO__H

#include <mutex>
#include "roboflex_core/core.h"
#include "roboflex_core/core_nodes/frequency_generator.h"
#include "hiwonder_bus_servo_controller.h"

namespace roboflex {
namespace hiwonderbusservonodes {

using namespace hiwonderbusservo;

constexpr char ModuleName[] = "hiwonder_bus_servo";

/**
 * Message carrying a snapshot of live state read back from one or more servos.
 *
 * This is the normal "state output" message for higher-level Hiwonder nodes.
 */
// Message carrying a snapshot of dynamic readback state.
class HiwonderBusServoGroupStateMessage : public core::Message {
public:
    inline static const char MessageName[] = "HBSGroupState";
    HiwonderBusServoGroupStateMessage(core::Message& other) : core::Message(other) {}
    HiwonderBusServoGroupStateMessage(const HiwonderBusServoGroupState& state);
    HiwonderBusServoGroupState get_state() const;
    void print_on(std::ostream& os) const override;

protected:
    mutable HiwonderBusServoGroupState state_;
    mutable bool state_initialized_ = false;
};

/**
 * Message carrying a grouped timed position command for one or more servos.
 *
 * This is the normal "command input" message for higher-level Hiwonder nodes.
 */
// Message carrying a grouped timed position command.
class HiwonderBusServoGroupCommandMessage : public core::Message {
public:
    inline static const char MessageName[] = "HBSGroupCommand";
    HiwonderBusServoGroupCommandMessage(core::Message& other) : core::Message(other) {}
    HiwonderBusServoGroupCommandMessage(const HiwonderBusServoGroupCommand& command);
    HiwonderBusServoGroupCommand get_command() const;
    void print_on(std::ostream& os) const override;

protected:
    mutable HiwonderBusServoGroupCommand command_;
    mutable bool command_initialized_ = false;
};

/**
 * Combined state-plus-command message emitted by local controller-style nodes.
 *
 * Useful when a node wants to publish both what it last observed and what it
 * decided to send in the same loop iteration.
 */
// Convenience message for "what we observed" and "what we commanded" in one packet.
class HiwonderBusServoStateCommandMessage : public core::Message {
public:
    inline static const char MessageName[] = "HBSStateCommand";
    HiwonderBusServoStateCommandMessage(core::Message& other) : core::Message(other) {}
    HiwonderBusServoStateCommandMessage(
        const HiwonderBusServoGroupState& state,
        const HiwonderBusServoGroupCommand& command);
    HiwonderBusServoGroupState get_state() const;
    HiwonderBusServoGroupCommand get_command() const;
    void print_on(std::ostream& os) const override;

protected:
    mutable HiwonderBusServoGroupState state_;
    mutable bool state_initialized_ = false;
    mutable HiwonderBusServoGroupCommand command_;
    mutable bool command_initialized_ = false;
};

/**
 * Base class for custom local controller nodes.
 *
 * This node owns a `HiwonderBusServoController`, runs its helper loop in a
 * background thread, and asks subclasses to produce the next command from the
 * latest observed servo state and the most recent upstream message.
 *
 * Use this when you want local feedback logic near the hardware.
 */
// Base class for custom controller nodes that own the servo loop locally.
class HiwonderBusServoGroupControllerNode : public core::RunnableNode {
public:
    HiwonderBusServoGroupControllerNode(
        HiwonderBusServoController::Ptr controller,
        DynamicReadConfig read_config,
        const std::string& name = "HBSGroupCtrl");

    HiwonderBusServoController::Ptr controller;
    DynamicReadConfig read_config;

    void receive(core::MessagePtr m) override;

protected:
    virtual HiwonderBusServoGroupCommand readwrite_loop_function(
        const HiwonderBusServoGroupState& state,
        const core::MessagePtr last_msg) = 0;

    void child_thread_fn() override;

    std::recursive_mutex last_message_mutex;
    core::MessagePtr last_message = nullptr;
};

/**
 * Simple local hardware node for command-forwarding use cases.
 *
 * It consumes each received `HiwonderBusServoGroupCommandMessage` at most once
 * and publishes `HiwonderBusServoGroupState` messages according to the
 * configured polling policy. This matches bus-servo usage much better than a
 * latched "re-send the last command forever" model.
 */
class HiwonderBusServoGroupNode : public core::RunnableNode {
public:
    HiwonderBusServoGroupNode(
        HiwonderBusServoController::Ptr controller,
        DynamicReadConfig read_config,
        const std::string& name = "HBSGroupNode");

    HiwonderBusServoController::Ptr controller;
    DynamicReadConfig read_config;

    void receive(core::MessagePtr m) override;

protected:
    bool readwrite_loop_function(
        const HiwonderBusServoGroupState& state,
        HiwonderBusServoGroupCommand& command);

    void child_thread_fn() override;

    std::recursive_mutex pending_command_message_mutex;
    std::shared_ptr<HiwonderBusServoGroupCommandMessage> pending_command_message = nullptr;
};

/**
 * Base class for remote-control patterns.
 *
 * This node receives state messages from a separate hardware-owning node and
 * responds by emitting command messages, but it does not access hardware itself.
 */
// Remote-control counterpart: receives state messages, emits command messages.
class HiwonderBusServoRemoteController : public core::Node {
public:
    HiwonderBusServoRemoteController(const std::string& name = "HBSRemoteCtrl");
    void receive(core::MessagePtr m) override;
    virtual HiwonderBusServoGroupCommand readwrite_loop_function(
        const HiwonderBusServoGroupState& state) = 0;
};

/**
 * Frequency-triggered version of HiwonderBusServoRemoteController.
 *
 * It keeps the latest known servo state and emits commands at a fixed rate,
 * which is useful when command generation should run at a stable control
 * frequency even if state messages arrive irregularly.
 */
// Same as HiwonderBusServoRemoteController, but triggered at a fixed frequency.
class HiwonderBusServoRemoteFrequencyController : public nodes::FrequencyGenerator {
public:
    HiwonderBusServoRemoteFrequencyController(
        float frequency_hz,
        const std::string& name = "HBSRemoteFreqCtrl");
    void receive(core::MessagePtr m) override;
    void on_trigger(double wall_clock_time) override;
    virtual HiwonderBusServoGroupCommand readwrite_loop_function(
        const HiwonderBusServoGroupState& state) = 0;

protected:
    HiwonderBusServoGroupState state_;
};

} // hiwonderbusservonodes
} // roboflex

#endif // ROBOFLEX_HIWONDER_BUS_SERVO__H
