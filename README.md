## Raspberry Pi Real-Time GPIO Control

Deterministic C++ control loop for Raspberry Pi that samples a distance sensor and drives a servo or ESC with hard timing guarantees. Ships with both a pigpio-backed hardware implementation and a desktop simulation, making it easy to tune the PID gains before deploying to an actual robot or industrial rig.

### Table of Contents

1. [Features](#features)
2. [Hardware Reference](#hardware-reference)
3. [Build Instructions](#build-instructions)
4. [Running the Controller](#running-the-controller)
5. [Deterministic Scheduling Tips](#deterministic-scheduling-tips)
6. [Repository Layout](#repository-layout)
7. [Extending the Project](#extending-the-project)

### Features

- 200 Hz PID loop enforced with absolute `clock_nanosleep` timing and optional `SCHED_FIFO` priority.
- Pluggable sensor/actuator interfaces (`ISensor` and `IActuator`) with pigpio hardware drivers and a rich simulator.
- pigpio-based HC-SR04 reader captures pulse widths with sub-microsecond precision; servo/ESC commands map PID output to 1000 us to 2000 us pulses.
- Command-line interface exposes setpoint, loop frequency, feedforward, gains, pin numbers, and RT priority.
- Console telemetry published on a background thread so the real-time loop remains deterministic.

### Hardware Reference

| Component        | BCM Pin | Notes                                                  |
|------------------|--------:|--------------------------------------------------------|
| HC-SR04 Trigger  | 23      | Configure as output                                    |
| HC-SR04 Echo     | 24      | Use 1k/2k resistor divider to clamp voltage to 3.3 V   |
| Servo/ESC signal | 18      | Hardware PWM capable (GPIO18 recommended)              |
| 5 V / GND rails  | 5 V, GND| Shared rails for sensor and actuator (watch current draw) |

The PID output is normalized to 0.0-1.0, then mapped to servo pulses between `--servo-min` and `--servo-max` (defaults: 1000 us, 2000 us). Adjust those bounds for your actuator.

### Build Instructions

```bash
sudo apt update
sudo apt install -y cmake g++ pigpio pigpiod
sudo systemctl enable --now pigpiod   # pigpiod must be running for pigpio clients

cmake -S . -B build -DENABLE_PIGPIO=ON
cmake --build build -j
```

Need a simulator-only build on a desktop that lacks pigpio headers? Disable the GPIO backend:

```bash
cmake -S . -B build -DENABLE_PIGPIO=OFF
cmake --build build -j
```

### Running the Controller

**Simulation mode (runs everywhere)**

```bash
./build/rt_control \
  --mode sim \
  --setpoint 40 \
  --kp 0.05 --ki 0.02 --kd 0.002 \
  --loop-hz 150
```

The simulated sensor produces a slow sinusoidal movement with Gaussian noise, letting you iterate on the PID gains. Console output looks like `distance_cm=36.2 command=0.54`.

**Hardware mode (Raspberry Pi + pigpio)**

```bash
sudo ./build/rt_control \
  --mode gpio \
  --setpoint 30 \
  --kp 0.08 --ki 0.02 --kd 0.005 \
  --loop-hz 200 \
  --priority 80 \
  --trig 23 --echo 24 --servo 18 \
  --servo-min 1000 --servo-max 2000
```

Run as root (or grant the binary `CAP_SYS_NICE`) so the control thread can request real-time scheduling. Use `--feedforward` to bias the neutral point for ESCs that expect a mid-stick value.

### Deterministic Scheduling Tips

- Install a PREEMPT_RT kernel (`sudo apt install raspberrypi-kernel-rt` or use `rpi-update` with an RT branch) for tighter jitter bounds.
- Pin the process to an isolated CPU (`isolcpus=3` in `/boot/cmdline.txt`, run with `taskset -c 3`).
- Force the performance governor: `sudo cpupower frequency-set -g performance`.
- Keep the real-time loop logging-free; telemetry already runs on a separate thread at 2 Hz.
- To measure jitter, run `sudo cyclictest -Sp90 -i200 -n -l100000` alongside the controller.

### Repository Layout

| Path                     | Description                                                  |
|--------------------------|--------------------------------------------------------------|
| `src/main.cpp`           | CLI parsing, signal handling, telemetry thread               |
| `src/controller.cpp`     | PID logic, absolute-timed loop, RT scheduling                |
| `src/gpio_devices.cpp`   | pigpio-backed HC-SR04 reader and servo driver                |
| `src/simulated_devices.cpp` | Sensor/actuator simulation for development                 |
| `include/`               | Public headers for the controller and device interfaces      |
| `docs/architecture.md`   | Timing and module design deep dive                           |

### Extending the Project

- Implement an `ISensor` for your IMU, temperature probe, or current sensor, and register it in `main.cpp`.
- Replace the servo with a BLDC ESC or H-bridge by writing a new `IActuator`.
- Publish telemetry to ROS2, log to CSV, or feed commands into a safety supervisor microcontroller.
- Add unit tests around `Controller::computePid` to validate tuning when you adjust gains.
