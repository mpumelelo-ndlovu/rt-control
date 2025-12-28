#pragma once

#include "controller.hpp"

#include <chrono>
#include <memory>
#include <random>

class SimulatedSensor : public ISensor {
public:
    SimulatedSensor();
    double read() override;

private:
    std::chrono::steady_clock::time_point start_time_;
    std::default_random_engine rng_;
    std::normal_distribution<double> noise_{0.0, 0.5};
};

class SimulatedActuator : public IActuator {
public:
    void apply(double normalized_command) override;
    double lastCommand() const { return last_command_; }

private:
    double last_command_ = 0.0;
};

#if defined(ENABLE_PIGPIO)
class PigpioManager {
public:
    PigpioManager();
    ~PigpioManager();
    bool isReady() const { return ready_; }

private:
    bool ready_ = false;
};

class HCSR04Sensor : public ISensor {
public:
    HCSR04Sensor(int trigger_pin, int echo_pin, unsigned timeout_us = 25'000);
    double read() override;

private:
    int trigger_pin_;
    int echo_pin_;
    unsigned timeout_us_;
    double last_good_reading_cm_ = 0.0;
};

class ServoActuator : public IActuator {
public:
    ServoActuator(int pwm_pin, unsigned min_pulse_us = 1000, unsigned max_pulse_us = 2000);
    void apply(double normalized_command) override;

private:
    int pwm_pin_;
    unsigned min_pulse_us_;
    unsigned max_pulse_us_;
};
#endif
