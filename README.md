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
