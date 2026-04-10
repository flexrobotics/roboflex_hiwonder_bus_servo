#ifndef ROBOFLEX_HIWONDER_BUS_SERVO_CONTROLLER__H
#define ROBOFLEX_HIWONDER_BUS_SERVO_CONTROLLER__H

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace roboflex {
namespace hiwonderbusservo {

using std::map;
using std::optional;
using std::pair;
using std::shared_ptr;
using std::string;
using std::vector;

using ServoId = uint8_t;

// Read loop timestamps in wall-clock seconds.
struct TimestampPair {
    double t0 = 0.0;
    double t1 = 0.0;
};

// Fields that are reasonable to poll repeatedly in the main loop.
enum class DynamicReadField : int {
    Position = 1,
    VoltageMV = 2,
    TemperatureC = 3,
    TorqueEnabled = 4,
};

// The live state we can read back for one servo over the board-mediated path.
struct HiwonderBusServoDynamicState {
    optional<int> position;
    optional<int> voltage_mv;
    optional<int> temperature_c;
    optional<bool> torque_enabled;

    void print_on(std::ostream& os) const;
    string to_string() const;
};

using HiwonderBusServoGroupValues = map<ServoId, HiwonderBusServoDynamicState>;

// Snapshot of dynamic state for a set of servos.
struct HiwonderBusServoGroupState {
    HiwonderBusServoGroupValues values;
    TimestampPair timestamp;

    void print_on(std::ostream& os) const;
    string to_string() const;
};

// A grouped timed position command. One duration applies to all positions.
struct HiwonderBusServoGroupCommand {
    map<ServoId, int> positions;
    int duration_ms = 20;
    bool should_write = false;
    TimestampPair timestamp;

    void set(ServoId servo_id, int position);
    void print_on(std::ostream& os) const;
    string to_string() const;
};

// Describes which live fields to poll and how often within the helper loop.
struct DynamicReadConfig {
    vector<ServoId> servo_ids;
    map<DynamicReadField, int> every_n_loops;
    int loop_sleep_ms = 0;

    void set_every_n_loops(DynamicReadField field, int every_n_loops_);
    vector<DynamicReadField> due_fields(size_t loop_index) const;
};

class HiwonderBusServoException : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * Low-level controller for the Hiwonder board-mediated bus-servo path.
 *
 * This class owns the serial connection to the robot controller board,
 * typically `/dev/rrc`, and implements the packet protocol used to issue
 * bus-servo commands through that board.
 *
 * You would use this class directly when you want explicit control over:
 * - grouped timed position writes
 * - one-servo-at-a-time dynamic reads such as position, voltage, and temperature
 * - one-off configuration reads and writes such as ID, offset, and limits
 *
 * This is the "transport and primitive operations" layer for the module.
 * It does not force a control policy: callers can use the direct methods
 * manually, or they can use `run_readwrite_loop(...)` to get a helper loop
 * that writes every iteration and polls selected live fields at configured
 * cadences.
 *
 * Compared with roboflex_dynamixel, this API is intentionally narrower.
 * The underlying hardware does not expose a rich register map or efficient
 * combined sync-read/sync-write operations, so this controller exposes the
 * operations the hardware can actually support cleanly.
 */
class HiwonderBusServoController {
public:
    using Ptr = shared_ptr<HiwonderBusServoController>;
    using ReadWriteLoopFunction = std::function<bool(
        const HiwonderBusServoGroupState&,
        HiwonderBusServoGroupCommand&)>;

    // Opens the board-facing serial device, typically /dev/rrc.
    HiwonderBusServoController(
        const string& device_name = "/dev/rrc",
        int baud_rate = 1000000,
        int timeout_ms = 100,
        int retries = 10);
    ~HiwonderBusServoController();

    string get_device_name() const { return device_name; }
    int get_baud_rate() const { return baud_rate; }
    int get_timeout_ms() const { return timeout_ms; }
    int get_retries() const { return retries; }

    // Fast path: one write updates any subset of servos with one shared duration.
    void write_positions(const HiwonderBusServoGroupCommand& command);
    void write_positions_ms(int duration_ms, const map<ServoId, int>& positions);
    void stop(const vector<ServoId>& servo_ids);

    // Dynamic reads expected to be useful in a polling loop.
    optional<int> read_position(ServoId servo_id);
    optional<int> read_voltage_mv(ServoId servo_id);
    optional<int> read_temperature_c(ServoId servo_id);
    optional<bool> read_torque_enabled(ServoId servo_id);
    HiwonderBusServoGroupState read_dynamic(
        const vector<ServoId>& servo_ids,
        const vector<DynamicReadField>& fields);

    // One-off config/state reads that are usually not worth polling continuously.
    optional<ServoId> read_id(ServoId servo_id = 254);
    optional<int> read_offset(ServoId servo_id);
    optional<pair<int, int>> read_angle_limit(ServoId servo_id);
    optional<pair<int, int>> read_vin_limit(ServoId servo_id);
    optional<int> read_temp_limit(ServoId servo_id);

    // One-off configuration writes.
    void set_torque_enabled(ServoId servo_id, bool enabled);
    void set_id(ServoId current_servo_id, ServoId new_servo_id);
    void set_offset(ServoId servo_id, int offset);
    void save_offset(ServoId servo_id);
    void set_angle_limit(ServoId servo_id, int min_position, int max_position);
    void set_vin_limit(ServoId servo_id, int min_mv, int max_mv);
    void set_temp_limit(ServoId servo_id, int max_temperature_c);

    void set_loop_sleep_ms(int milliseconds) { loop_sleep_ms = milliseconds; }
    int get_loop_sleep_ms() const { return loop_sleep_ms; }

    // Optional helper loop: writes every iteration, polls only configured fields when due.
    void run_readwrite_loop(
        const DynamicReadConfig& read_config,
        const ReadWriteLoopFunction& readwrite_loop_function);

    // Present for parity with other Roboflex modules; no-op here.
    void freeze();

private:
    class Impl;
    std::unique_ptr<Impl> impl;

    string device_name;
    int baud_rate;
    int timeout_ms;
    int retries;
    int loop_sleep_ms = 0;
};

} // hiwonderbusservo
} // roboflex

#endif // ROBOFLEX_HIWONDER_BUS_SERVO_CONTROLLER__H
