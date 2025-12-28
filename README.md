## Raspberry Pi Real-Time GPIO Control

> Deterministic C++ control loop for Raspberry Pi that samples a distance sensor and drives a servo or ESC with hard timing guarantees. Build locally with a simulator, deploy to pigpio hardware when ready.

| At a Glance              | Details                                                                    |
|--------------------------|----------------------------------------------------------------------------|
| Loop frequency           | 200 Hz (configurable)                                                      |
| Timing strategy          | Absolute `clock_nanosleep`, optional `SCHED_FIFO` priority                 |
| Sensors/Actuators        | HC-SR04 ultrasonic + servo/ESC by default (easy to extend)                 |
| Simulator                | Sinusoidal distance model with Gaussian noise                             |
| Build system             | CMake + pigpio (optional)                                                  |
| Target boards            | Raspberry Pi 3/4/5, CM4, Zero 2 (anything with pigpio + Linux)            |

### Table of Contents

1. [Features](#features)
2. [Hardware Reference](#hardware-reference)
3. [Build Instructions](#build-instructions)
4. [Running the Controller](#running-the-controller)
5. [Use Case: Lab Fan Stabilizer](#use-case-lab-fan-stabilizer)
6. [Why I Built This](#why-i-built-this)
7. [Deterministic Scheduling Tips](#deterministic-scheduling-tips)
8. [Repository Layout](#repository-layout)
9. [Extending the Project](#extending-the-project)

### Features

- **Deterministic loop core**: Absolute-time sleeping removes drift even on long runtimes.
- **Real-time friendly**: Optional `SCHED_FIFO` priority and `pthread` affinity ready for PREEMPT_RT.
- **Noise-aware sensing**: Built-in exponential moving average smooths HC-SR04 jitter; disable or retune via CLI.
- **Desktop simulator**: Tune PID gains and verify telemetry on any Linux/macOS/Windows box.
- **Configurable CLI**: Override setpoint, gains, feedforward, pins, PWM pulse lengths, and telemetry cadence at runtime.
- **Telemetry-safe**: Console updates and CSV logging run on a secondary thread so the control loop never blocks on I/O.
- **Flight data recorder**: `--log-file measurements.csv` dumps timestamped measurements/commands for post-run analysis.

### Hardware Reference

```
          +-----------------------------+         +----------------+
          |        Raspberry Pi         |         |   HC-SR04      |
          |                             |         |                |
5 V  ---- | 5V -------------------------+---------| VCC            |
GND  ---- | GND ------------------------+---------| GND            |
GPIO23 -- | TRIG -------------------------------+-| TRIG           |
GPIO24 <- | ECHO <-- voltage divider (1k/2k) <--+-| ECHO           |
GPIO18 -> | SERVO PWM ---------------------------| Signal          |
          +-----------------------------+         +----------------+
```

| Component        | BCM Pin | Notes                                                  |
|------------------|--------:|--------------------------------------------------------|
| HC-SR04 Trigger  | 23      | Configure as output                                    |
| HC-SR04 Echo     | 24      | Use 1k/2k resistor divider to clamp voltage to 3.3 V   |
| Servo/ESC signal | 18      | Hardware PWM capable (GPIO18 recommended)              |
| 5 V / GND rails  | 5 V, GND| Shared rails for sensor and actuator (watch current draw) |

The PID output is normalized to 0.0-1.0, then mapped to servo pulses between `--servo-min` and `--servo-max` (defaults: 1000 us, 2000 us). Adjust those bounds for your actuator, ESC, or fan controller.

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

**Quick Start**

1. Build the project (`cmake -S . -B build …; cmake --build build`).
2. Start the desired sensor/actuator backend via CLI flag.
3. Observe telemetry in the console or pipe it elsewhere.

**Simulation mode (runs everywhere)**

```bash
./build/rt_control \
  --mode sim \
  --setpoint 40 \
  --kp 0.05 --ki 0.02 --kd 0.002 \
  --loop-hz 150
```

The simulated sensor produces a slow sinusoidal movement with Gaussian noise, letting you iterate on the PID gains. Console output looks like:

```
distance_cm=36.2 command=0.54
distance_cm=34.9 command=0.51
```

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

**Command Reference**

| Flag                  | Purpose                                             | Default |
|-----------------------|-----------------------------------------------------|---------|
| `--mode`              | `sim` (desktop) or `gpio` (pigpio hardware)         | `sim`   |
| `--setpoint`          | Desired distance in centimeters                     | `30`    |
| `--kp`, `--ki`, `--kd`| PID gains                                           | `0.08`, `0.02`, `0.005` |
| `--loop-hz`           | Control loop frequency                              | `200`   |
| `--priority`          | Linux realtime priority (1–99)                      | `80`    |
| `--feedforward`       | Neutral offset for actuators                        | `0.5`   |
| `--filter-alpha`      | Exponential moving average coefficient (0 disables) | `0.25`  |
| `--trig`, `--echo`, `--servo` | BCM pins for devices                        | `23`, `24`, `18` |
| `--servo-min`, `--servo-max` | PWM bounds in microseconds                    | `1000`, `2000` |
| `--log-file path`     | CSV telemetry output (`timestamp_us,distance,cmd`)  | _off_   |
| `--telemetry-ms`      | Console/logging interval in milliseconds            | `500`   |

### Use Case: Lab Fan Stabilizer

I originally piloted this code on a tabletop rig that used an HC-SR04 pointed at a foam baffle mounted to a desktop fan. By measuring the baffle distance and modulating the servo controlling the fan’s intake flap, I could keep airflow constant even when people walked past the desk or when ambient pressure changed. The same loop has since been adapted for:

- A mini line-following robot that keeps its ultrasonic sensor at a fixed standoff from walls.
- An industrial demonstrator that positions a pneumatic valve to stabilize tank pressure.
- Classroom demos where we replay sensor recordings through the simulator to show how PID gains affect settling time.

### Why I Built This

As an CompSci student who is into systems development, I needed a portfolio piece that proved I could juggle firmware, Linux scheduling, and hardware bring-up. 
I also wanted hands-on practice with PREEMPT_RT, pigpio, and deterministic loops beyond microcontroller Arduino sketches.
Feel free to reuse this framework if you are also showcasing embedded/robotics chops—just swap in your own sensor/actuator pair and document the results.

### Deterministic Scheduling Tips

- Install a PREEMPT_RT kernel (`sudo apt install raspberrypi-kernel-rt` or use `rpi-update` with an RT branch) for tighter jitter bounds.
- Pin the process to an isolated CPU (`isolcpus=3` in `/boot/cmdline.txt`, run with `taskset -c 3`).
- Force the performance governor: `sudo cpupower frequency-set -g performance`.
- Keep the real-time loop logging-free; telemetry already runs on a separate thread at 2 Hz.
- To measure jitter, run `sudo cyclictest -Sp90 -i200 -n -l100000` alongside the controller.
- On PREEMPT_RT, set `--priority` just below other critical services (e.g., 80) to avoid priority inversion.
- Disable background daemons (Bluetooth, Wi-Fi, GUI) when benchmarking jitter-sensitive workloads.

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
- Introduce a `--config path.toml` loader to bundle multiple parameters per robot profile.
- Wrap the binary with a systemd service that sets CPU affinity and governor before launch.
