## Real-Time GPIO Control Overview

The controller runs as a single high-priority thread pinned to a deterministic loop interval. Each iteration performs the following sequence:

1. Trigger distance measurement on an HC-SR04 ultrasonic sensor by emitting a 10 µs pulse on the `TRIG` pin and timing the echo pulse on the `ECHO` pin using pigpio's timestamped callbacks.
2. Convert the measured echo duration to distance (cm) and feed it into a PID controller that compares it with the configured setpoint.
3. Drive a servo (or ESC controlling a DC motor) via pigpio hardware PWM, mapping the PID output to an impulse width between 1000 µs and 2000 µs.

### Timing Strategy

- The control loop targets a 200 Hz cycle (~5 ms) enforced with `clock_nanosleep` and absolute timing to avoid drift.
- The working thread elevates itself to `SCHED_FIFO` with configurable priority. On kernels with PREEMPT_RT, this results in bounded latency; on stock Raspberry Pi OS it still reduces jitter significantly.
- Sensor sampling and actuator updates only occur inside this loop, so the sensor latency is at most one cycle.
- A configurable exponential moving average smooths raw distance measurements before they feed the PID block. Setting `measurement_filter_alpha` to zero disables filtering when raw response is required.

### Modularity

The code splits interfaces to decouple I/O specifics:

- `ISensor`: abstracts the data acquisition (`Read()` returns the last measured distance).
- `IActuator`: handles PWM output given a normalized command.
- `Controller`: owns the PID state, timing loop, and delegates to the interfaces.

For development on non-Raspberry Pi hardware, the simulator builds with stub sensor/actuator implementations that replay recorded CSV data, allowing logic testing without GPIO.

### Telemetry and Logging

A detached status thread samples the controller’s latest measurement/command at the user-defined cadence. It prints to the console and, when `--log-file` is supplied, streams a CSV (`timestamp_us,distance_cm,command`) so traces can be plotted in MATLAB, Python, or a spreadsheet without disturbing the real-time loop.
