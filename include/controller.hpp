#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>

struct ControlConfig {
    double setpoint_cm = 30.0;
    double kp = 0.08;
    double ki = 0.02;
    double kd = 0.005;
    double loop_hz = 200.0;
    double max_integral = 100.0;
    double feedforward = 0.5;
    double actuator_min = 0.0;
    double actuator_max = 1.0;
    int realtime_priority = 80;
};

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual double read() = 0;  // returns centimeters
};

class IActuator {
public:
    virtual ~IActuator() = default;
    virtual void apply(double normalized_command) = 0;  // expects value in [0, 1]
};

class Controller {
public:
    Controller(ISensor& sensor, IActuator& actuator, ControlConfig config);
    ~Controller();

    void start();
    void stop();
    bool running() const { return running_.load(); }

    double latestMeasurement() const { return last_measurement_.load(); }
    double latestCommand() const { return last_command_.load(); }

private:
    void loop();
    double computePid(double measurement, double dt_seconds);

    ISensor& sensor_;
    IActuator& actuator_;
    ControlConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
    std::atomic<double> last_measurement_{0.0};
    std::atomic<double> last_command_{0.0};
    double integral_state_ = 0.0;
    double previous_error_ = 0.0;
};
