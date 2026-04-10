# roboflex_hiwonder_bus_servo

Native Roboflex support for the Hiwonder controller-board bus-servo protocol used by ArmPi-FPV style robots.

This module intentionally targets the validated subset of the hardware:

- grouped timed position writes
- live reads for present position, voltage, temperature, and torque-enabled state
- one-off reads/writes for ID, offset, and safety limits
- an optional read/write loop with configurable dynamic polling cadence

Unlike Dynamixel, this hardware does not expose a rich register map or efficient combined sync-read/sync-write transactions. Writes are grouped; reads are largely request/response per-servo.

## Build

Build the C++ library only:

```bash
cmake -S . -B build -DBUILD_ROBOFLEX_PYTHON_EXT=OFF
cmake --build build -j
```

Build the C++ library and Python wrapper:

```bash
cmake -S . -B build
cmake --build build -j
```

Build and install the Python package with the extension via `setup.py`:

```bash
python3 setup.py build
python3 setup.py install
```

Or with `pip`:

```bash
python3 -m pip install .
```

Notes:

- The top-level CMake build always builds the core C++ library.
- The Python wrapper is built by the `python/` subdirectory.
- This package prefers the sibling local `../roboflex` checkout when present.

## Usage

Basic Python usage:

```python
import roboflex.hiwonder_bus_servo as rhs

controller = rhs.HiwonderBusServoController("/dev/rrc", 1000000)

# Move two servos together over 100 ms.
controller.write_positions_ms(100, {
    1: 300,
    2: 700,
})

# Read back a few dynamic values.
position = controller.read_position(1)
voltage_mv = controller.read_voltage_mv(1)
temperature_c = controller.read_temperature_c(1)

print(position, voltage_mv, temperature_c)
```

Python helper-loop usage:

```python
import roboflex.hiwonder_bus_servo as rhs

controller = rhs.HiwonderBusServoController("/dev/rrc", 1000000)

config = rhs.DynamicReadConfig()
config.servo_ids = [1, 2]
config.set_every_n_loops(rhs.DynamicReadField.Position, 1)
config.set_every_n_loops(rhs.DynamicReadField.VoltageMV, 20)
config.loop_sleep_ms = 20

def loop(state, command):
    command.duration_ms = 100
    command.should_write = True
    command.positions = {1: 350, 2: 650}
    print(state)
    return True

controller.run_readwrite_loop(config, loop)
```

Basic C++ usage:

```cpp
#include <iostream>
#include "roboflex_hiwonder_bus_servo/hiwonder_bus_servo_controller.h"

using namespace roboflex::hiwonderbusservo;

int main() {
    auto controller = std::make_shared<HiwonderBusServoController>("/dev/rrc", 1000000);

    controller->write_positions_ms(100, {
        {1, 300},
        {2, 700},
    });

    auto position = controller->read_position(1);
    auto voltage_mv = controller->read_voltage_mv(1);

    if (position.has_value()) {
        std::cout << "position: " << *position << std::endl;
    }
    if (voltage_mv.has_value()) {
        std::cout << "voltage_mv: " << *voltage_mv << std::endl;
    }

    return 0;
}
```

The main pattern is:

- Use `write_positions_ms(...)` for fast grouped timed moves.
- Use `read_position(...)`, `read_voltage_mv(...)`, `read_temperature_c(...)`, and `read_torque_enabled(...)` for live reads.
- Use the helper loop only when you want continuous writing plus configurable polling in one place.
