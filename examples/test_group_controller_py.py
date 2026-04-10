import time
import roboflex as rf
import roboflex.hiwonder_bus_servo as rhs


controller = rhs.HiwonderBusServoController("/dev/rrc", 1000000, 100, 5)

config = rhs.DynamicReadConfig()
config.servo_ids = [1, 2]
config.set_every_n_loops(rhs.DynamicReadField.Position, 1)
config.set_every_n_loops(rhs.DynamicReadField.VoltageMV, 20)
config.set_every_n_loops(rhs.DynamicReadField.TemperatureC, 20)
config.loop_sleep_ms = 20


def loop(state, command):
    print(state)
    t = time.time()
    position = 300 if int(t) % 2 == 0 else 700
    command.duration_ms = 100
    command.should_write = True
    command.positions = {1: position, 2: 1000 - position}
    return True


controller.run_readwrite_loop(config, loop)
