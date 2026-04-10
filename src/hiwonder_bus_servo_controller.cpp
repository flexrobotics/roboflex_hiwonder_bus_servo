#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <optional>
#include <poll.h>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <vector>
#include <termios.h>

#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

#include "roboflex_hiwonder_bus_servo/hiwonder_bus_servo_controller.h"

namespace roboflex {
namespace hiwonderbusservo {

namespace {

constexpr uint8_t kStartByte1 = 0xAA;
constexpr uint8_t kStartByte2 = 0x55;
constexpr uint8_t kPacketFuncBusServo = 5;

constexpr uint8_t kCmdSetPosition = 0x01;
constexpr uint8_t kCmdStop = 0x03;
constexpr uint8_t kCmdReadPosition = 0x05;
constexpr uint8_t kCmdReadVoltage = 0x07;
constexpr uint8_t kCmdReadTemperature = 0x09;
constexpr uint8_t kCmdEnableTorque = 0x0B;
constexpr uint8_t kCmdDisableTorque = 0x0C;
constexpr uint8_t kCmdReadTorqueState = 0x0D;
constexpr uint8_t kCmdSetId = 0x10;
constexpr uint8_t kCmdReadId = 0x12;
constexpr uint8_t kCmdSetOffset = 0x20;
constexpr uint8_t kCmdReadOffset = 0x22;
constexpr uint8_t kCmdSaveOffset = 0x24;
constexpr uint8_t kCmdSetAngleLimit = 0x30;
constexpr uint8_t kCmdReadAngleLimit = 0x32;
constexpr uint8_t kCmdSetVinLimit = 0x34;
constexpr uint8_t kCmdReadVinLimit = 0x36;
constexpr uint8_t kCmdSetTempLimit = 0x38;
constexpr uint8_t kCmdReadTempLimit = 0x3A;

// CRC table used by the board-facing /dev/rrc protocol.
constexpr std::array<uint8_t, 256> kCrc8Table = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

double wall_time_seconds() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

uint8_t checksum_crc8(const std::vector<uint8_t>& data) {
    uint8_t check = 0;
    for (uint8_t b : data) {
        check = kCrc8Table[check ^ b];
    }
    return check;
}

void append_u16_le(std::vector<uint8_t>& data, int value) {
    uint16_t v = static_cast<uint16_t>(value);
    data.push_back(static_cast<uint8_t>(v & 0xFF));
    data.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

int read_u16_le(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 1 >= data.size()) {
        throw HiwonderBusServoException("Attempted to read u16 past payload boundary");
    }
    return static_cast<int>(data[offset]) | (static_cast<int>(data[offset + 1]) << 8);
}

int read_i16_le(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int16_t>(read_u16_le(data, offset));
}

int clamp_position(int position) {
    return std::clamp(position, 0, 1000);
}

int clamp_duration_ms(int duration_ms) {
    return std::clamp(duration_ms, 0, 30000);
}

string dynamic_read_field_to_string(DynamicReadField field) {
    switch (field) {
        case DynamicReadField::Position:
            return "position";
        case DynamicReadField::VoltageMV:
            return "voltage_mv";
        case DynamicReadField::TemperatureC:
            return "temperature_c";
        case DynamicReadField::TorqueEnabled:
            return "torque_enabled";
    }
    return "unknown";
}

} // namespace

void HiwonderBusServoDynamicState::print_on(std::ostream& os) const {
    os << "{";
    bool first = true;
    auto write_sep = [&]() {
        if (!first) {
            os << ", ";
        }
        first = false;
    };
    if (position.has_value()) {
        write_sep();
        os << "position: " << *position;
    }
    if (voltage_mv.has_value()) {
        write_sep();
        os << "voltage_mv: " << *voltage_mv;
    }
    if (temperature_c.has_value()) {
        write_sep();
        os << "temperature_c: " << *temperature_c;
    }
    if (torque_enabled.has_value()) {
        write_sep();
        os << "torque_enabled: " << (*torque_enabled ? "true" : "false");
    }
    os << "}";
}

string HiwonderBusServoDynamicState::to_string() const {
    std::stringstream sst;
    print_on(sst);
    return sst.str();
}

void HiwonderBusServoGroupState::print_on(std::ostream& os) const {
    os << "{";
    bool first = true;
    for (const auto& [servo_id, state] : values) {
        if (!first) {
            os << ", ";
        }
        first = false;
        os << static_cast<int>(servo_id) << ": ";
        state.print_on(os);
    }
    os << "} t0: " << std::fixed << std::setprecision(3) << timestamp.t0
       << " t1: " << timestamp.t1;
}

string HiwonderBusServoGroupState::to_string() const {
    std::stringstream sst;
    sst << "<HiwonderBusServoGroupState ";
    print_on(sst);
    sst << ">";
    return sst.str();
}

void HiwonderBusServoGroupCommand::set(ServoId servo_id, int position) {
    positions[servo_id] = clamp_position(position);
}

void HiwonderBusServoGroupCommand::print_on(std::ostream& os) const {
    os << "{duration_ms: " << duration_ms << ", should_write: " << should_write << ", positions: {";
    bool first = true;
    for (const auto& [servo_id, position] : positions) {
        if (!first) {
            os << ", ";
        }
        first = false;
        os << static_cast<int>(servo_id) << ": " << position;
    }
    os << "}} t0: " << std::fixed << std::setprecision(3) << timestamp.t0
       << " t1: " << timestamp.t1;
}

string HiwonderBusServoGroupCommand::to_string() const {
    std::stringstream sst;
    sst << "<HiwonderBusServoGroupCommand ";
    print_on(sst);
    sst << ">";
    return sst.str();
}

void DynamicReadConfig::set_every_n_loops(DynamicReadField field, int every_n_loops_) {
    every_n_loops[field] = std::max(1, every_n_loops_);
}

vector<DynamicReadField> DynamicReadConfig::due_fields(size_t loop_index) const {
    vector<DynamicReadField> result;
    for (const auto& [field, every] : every_n_loops) {
        if (every > 0 && (loop_index % static_cast<size_t>(every) == 0)) {
            result.push_back(field);
        }
    }
    return result;
}

class HiwonderBusServoController::Impl {
public:
    Impl(const string& device_name_, int baud_rate_, int timeout_ms_, int retries_);
    ~Impl();

    // Board protocol framing: 0xAA 0x55, function id, payload length, payload, crc8.
    void write_frame(uint8_t function_id, const std::vector<uint8_t>& payload);
    std::pair<uint8_t, std::vector<uint8_t>> read_frame(int timeout_ms);
    optional<std::vector<uint8_t>> transact_bus_servo_read(ServoId servo_id, uint8_t cmd);
    void sleep_after_write() const;

    int fd = -1;
    int timeout_ms;
    int retries;
    std::mutex io_mutex;
};

HiwonderBusServoController::Impl::Impl(
    const string& device_name_,
    int baud_rate_,
    int timeout_ms_,
    int retries_) :
        timeout_ms(timeout_ms_),
        retries(retries_) {
    fd = ::open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        throw HiwonderBusServoException("Unable to open serial device " + device_name_);
    }

    termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        throw HiwonderBusServoException("Unable to read serial attributes for " + device_name_);
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

#if defined(__APPLE__)
    speed_t fallback_speed = B115200;
    cfsetispeed(&tty, fallback_speed);
    cfsetospeed(&tty, fallback_speed);
#elif defined(B1000000)
    cfsetispeed(&tty, B1000000);
    cfsetospeed(&tty, B1000000);
#else
    ::close(fd);
    throw HiwonderBusServoException("This platform does not expose B1000000");
#endif

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        throw HiwonderBusServoException("Unable to configure serial device");
    }

#if defined(__APPLE__)
    speed_t speed = static_cast<speed_t>(baud_rate_);
    if (ioctl(fd, IOSSIOSPEED, &speed) == -1) {
        ::close(fd);
        throw HiwonderBusServoException("Unable to set custom baud rate");
    }
#endif
}

HiwonderBusServoController::Impl::~Impl() {
    if (fd >= 0) {
        ::close(fd);
    }
}

void HiwonderBusServoController::Impl::write_frame(
    uint8_t function_id,
    const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 5);
    frame.push_back(kStartByte1);
    frame.push_back(kStartByte2);
    frame.push_back(function_id);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());

    std::vector<uint8_t> crc_input(frame.begin() + 2, frame.end());
    frame.push_back(checksum_crc8(crc_input));

    ssize_t written = ::write(fd, frame.data(), frame.size());
    if (written < 0 || static_cast<size_t>(written) != frame.size()) {
        throw HiwonderBusServoException("Serial write failed");
    }

    tcdrain(fd);
}

std::pair<uint8_t, std::vector<uint8_t>> HiwonderBusServoController::Impl::read_frame(int timeout_ms_) {
    enum class ParseState {
        Start1,
        Start2,
        Function,
        Length,
        Payload,
        Checksum,
    };

    ParseState state = ParseState::Start1;
    uint8_t function_id = 0;
    uint8_t payload_length = 0;
    std::vector<uint8_t> frame_without_prefix;
    std::vector<uint8_t> payload;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        pollfd pfd {fd, POLLIN, 0};
        int poll_result = ::poll(&pfd, 1, static_cast<int>(std::max<int64_t>(remaining, 1)));
        if (poll_result < 0) {
            throw HiwonderBusServoException("Serial poll failed");
        }
        if (poll_result == 0) {
            break;
        }

        uint8_t byte = 0;
        ssize_t bytes_read = ::read(fd, &byte, 1);
        if (bytes_read <= 0) {
            continue;
        }

        switch (state) {
            case ParseState::Start1:
                if (byte == kStartByte1) {
                    state = ParseState::Start2;
                }
                break;
            case ParseState::Start2:
                if (byte == kStartByte2) {
                    state = ParseState::Function;
                } else {
                    state = ParseState::Start1;
                }
                break;
            case ParseState::Function:
                function_id = byte;
                frame_without_prefix.clear();
                frame_without_prefix.push_back(byte);
                state = ParseState::Length;
                break;
            case ParseState::Length:
                payload_length = byte;
                frame_without_prefix.push_back(byte);
                payload.clear();
                state = payload_length == 0 ? ParseState::Checksum : ParseState::Payload;
                break;
            case ParseState::Payload:
                payload.push_back(byte);
                frame_without_prefix.push_back(byte);
                if (payload.size() >= payload_length) {
                    state = ParseState::Checksum;
                }
                break;
            case ParseState::Checksum: {
                uint8_t expected = checksum_crc8(frame_without_prefix);
                if (expected == byte) {
                    return {function_id, payload};
                }
                state = ParseState::Start1;
                break;
            }
        }
    }

    return std::pair<uint8_t, std::vector<uint8_t>>{0, {}};
}

optional<std::vector<uint8_t>> HiwonderBusServoController::Impl::transact_bus_servo_read(
    ServoId servo_id,
    uint8_t cmd) {
    std::lock_guard<std::mutex> lock(io_mutex);
    std::vector<uint8_t> payload {cmd, servo_id};

    for (int attempt = 0; attempt <= retries; ++attempt) {
        write_frame(kPacketFuncBusServo, payload);
        auto [function_id, response] = read_frame(timeout_ms);
        if (function_id != kPacketFuncBusServo || response.size() < 3) {
            continue;
        }
        if (response[1] != cmd) {
            continue;
        }
        if (response[2] != 0) {
            return std::nullopt;
        }
        return response;
    }

    return std::nullopt;
}

void HiwonderBusServoController::Impl::sleep_after_write() const {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

HiwonderBusServoController::HiwonderBusServoController(
    const string& device_name,
    int baud_rate,
    int timeout_ms,
    int retries) :
        impl(std::make_unique<Impl>(device_name, baud_rate, timeout_ms, retries)),
        device_name(device_name),
        baud_rate(baud_rate),
        timeout_ms(timeout_ms),
        retries(retries) {
}

HiwonderBusServoController::~HiwonderBusServoController() = default;

void HiwonderBusServoController::write_positions(const HiwonderBusServoGroupCommand& command) {
    if (!command.should_write) {
        return;
    }
    write_positions_ms(command.duration_ms, command.positions);
}

void HiwonderBusServoController::write_positions_ms(
    int duration_ms,
    const map<ServoId, int>& positions) {
    if (positions.empty()) {
        return;
    }

    std::vector<uint8_t> payload;
    payload.reserve(4 + positions.size() * 3);
    payload.push_back(kCmdSetPosition);
    duration_ms = clamp_duration_ms(duration_ms);
    append_u16_le(payload, duration_ms);
    payload.push_back(static_cast<uint8_t>(positions.size()));
    for (const auto& [servo_id, position] : positions) {
        payload.push_back(servo_id);
        append_u16_le(payload, clamp_position(position));
    }

    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
}

void HiwonderBusServoController::stop(const vector<ServoId>& servo_ids) {
    if (servo_ids.empty()) {
        return;
    }

    std::vector<uint8_t> payload;
    payload.reserve(2 + servo_ids.size());
    payload.push_back(kCmdStop);
    payload.push_back(static_cast<uint8_t>(servo_ids.size()));
    payload.insert(payload.end(), servo_ids.begin(), servo_ids.end());

    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
}

optional<int> HiwonderBusServoController::read_position(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadPosition);
    if (!response || response->size() < 5) {
        return std::nullopt;
    }
    return read_i16_le(*response, 3);
}

optional<int> HiwonderBusServoController::read_voltage_mv(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadVoltage);
    if (!response || response->size() < 5) {
        return std::nullopt;
    }
    return read_u16_le(*response, 3);
}

optional<int> HiwonderBusServoController::read_temperature_c(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadTemperature);
    if (!response || response->size() < 4) {
        return std::nullopt;
    }
    return static_cast<int>((*response)[3]);
}

optional<bool> HiwonderBusServoController::read_torque_enabled(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadTorqueState);
    if (!response || response->size() < 4) {
        return std::nullopt;
    }
    return (*response)[3] != 0;
}

HiwonderBusServoGroupState HiwonderBusServoController::read_dynamic(
    const vector<ServoId>& servo_ids,
    const vector<DynamicReadField>& fields) {
    HiwonderBusServoGroupState state;
    state.timestamp.t0 = wall_time_seconds();

    for (ServoId servo_id : servo_ids) {
        auto& servo_state = state.values[servo_id];
        for (DynamicReadField field : fields) {
            switch (field) {
                case DynamicReadField::Position:
                    servo_state.position = read_position(servo_id);
                    break;
                case DynamicReadField::VoltageMV:
                    servo_state.voltage_mv = read_voltage_mv(servo_id);
                    break;
                case DynamicReadField::TemperatureC:
                    servo_state.temperature_c = read_temperature_c(servo_id);
                    break;
                case DynamicReadField::TorqueEnabled:
                    servo_state.torque_enabled = read_torque_enabled(servo_id);
                    break;
            }
        }
    }

    state.timestamp.t1 = wall_time_seconds();
    return state;
}

optional<ServoId> HiwonderBusServoController::read_id(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadId);
    if (!response || response->size() < 4) {
        return std::nullopt;
    }
    return (*response)[3];
}

optional<int> HiwonderBusServoController::read_offset(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadOffset);
    if (!response || response->size() < 4) {
        return std::nullopt;
    }
    return static_cast<int>(static_cast<int8_t>((*response)[3]));
}

optional<pair<int, int>> HiwonderBusServoController::read_angle_limit(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadAngleLimit);
    if (!response || response->size() < 7) {
        return std::nullopt;
    }
    return pair<int, int>{read_u16_le(*response, 3), read_u16_le(*response, 5)};
}

optional<pair<int, int>> HiwonderBusServoController::read_vin_limit(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadVinLimit);
    if (!response || response->size() < 7) {
        return std::nullopt;
    }
    return pair<int, int>{read_u16_le(*response, 3), read_u16_le(*response, 5)};
}

optional<int> HiwonderBusServoController::read_temp_limit(ServoId servo_id) {
    auto response = impl->transact_bus_servo_read(servo_id, kCmdReadTempLimit);
    if (!response || response->size() < 4) {
        return std::nullopt;
    }
    return static_cast<int>((*response)[3]);
}

void HiwonderBusServoController::set_torque_enabled(ServoId servo_id, bool enabled) {
    std::vector<uint8_t> payload {enabled ? kCmdEnableTorque : kCmdDisableTorque, servo_id};
    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
    impl->sleep_after_write();
}

void HiwonderBusServoController::set_id(ServoId current_servo_id, ServoId new_servo_id) {
    std::vector<uint8_t> payload {kCmdSetId, current_servo_id, new_servo_id};
    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
    impl->sleep_after_write();
}

void HiwonderBusServoController::set_offset(ServoId servo_id, int offset) {
    std::vector<uint8_t> payload {
        kCmdSetOffset,
        servo_id,
        static_cast<uint8_t>(static_cast<int8_t>(offset))
    };
    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
    impl->sleep_after_write();
}

void HiwonderBusServoController::save_offset(ServoId servo_id) {
    std::vector<uint8_t> payload {kCmdSaveOffset, servo_id};
    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
    impl->sleep_after_write();
}

void HiwonderBusServoController::set_angle_limit(
    ServoId servo_id,
    int min_position,
    int max_position) {
    std::vector<uint8_t> payload {kCmdSetAngleLimit, servo_id};
    append_u16_le(payload, clamp_position(min_position));
    append_u16_le(payload, clamp_position(max_position));
    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
    impl->sleep_after_write();
}

void HiwonderBusServoController::set_vin_limit(
    ServoId servo_id,
    int min_mv,
    int max_mv) {
    std::vector<uint8_t> payload {kCmdSetVinLimit, servo_id};
    append_u16_le(payload, min_mv);
    append_u16_le(payload, max_mv);
    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
    impl->sleep_after_write();
}

void HiwonderBusServoController::set_temp_limit(ServoId servo_id, int max_temperature_c) {
    std::vector<uint8_t> payload {
        kCmdSetTempLimit,
        servo_id,
        static_cast<uint8_t>(max_temperature_c)
    };
    std::lock_guard<std::mutex> lock(impl->io_mutex);
    impl->write_frame(kPacketFuncBusServo, payload);
    impl->sleep_after_write();
}

void HiwonderBusServoController::run_readwrite_loop(
    const DynamicReadConfig& read_config,
    const ReadWriteLoopFunction& readwrite_loop_function) {
    HiwonderBusServoGroupState cached_state;
    HiwonderBusServoGroupCommand command;
    size_t loop_index = 0;

    // This loop is intentionally asymmetric: writes can happen every pass,
    // while reads are scheduled independently because they are much costlier.
    while (readwrite_loop_function(cached_state, command)) {
        double t0 = wall_time_seconds();

        if (command.should_write && !command.positions.empty()) {
            write_positions(command);
        }

        auto due = read_config.due_fields(loop_index);
        if (!due.empty() && !read_config.servo_ids.empty()) {
            auto partial = read_dynamic(read_config.servo_ids, due);
            for (const auto& [servo_id, servo_state] : partial.values) {
                auto& cached = cached_state.values[servo_id];
                if (servo_state.position.has_value()) {
                    cached.position = servo_state.position;
                }
                if (servo_state.voltage_mv.has_value()) {
                    cached.voltage_mv = servo_state.voltage_mv;
                }
                if (servo_state.temperature_c.has_value()) {
                    cached.temperature_c = servo_state.temperature_c;
                }
                if (servo_state.torque_enabled.has_value()) {
                    cached.torque_enabled = servo_state.torque_enabled;
                }
            }
        }

        cached_state.timestamp = {t0, wall_time_seconds()};
        ++loop_index;

        int sleep_ms = read_config.loop_sleep_ms > 0 ? read_config.loop_sleep_ms : loop_sleep_ms;
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }
}

void HiwonderBusServoController::freeze() {
}

} // hiwonderbusservo
} // roboflex
