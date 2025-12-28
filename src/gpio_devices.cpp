#include "devices.hpp"

#if defined(ENABLE_PIGPIO)
#include <pigpio.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>

namespace {
uint32_t tickDiff(uint32_t start, uint32_t end) {
    if (end >= start) {
        return end - start;
    }
    return (0xFFFFFFFFu - start) + end + 1u;
}
}  // namespace

PigpioManager::PigpioManager() {
    const int rc = gpioInitialise();
    if (rc < 0) {
        throw std::runtime_error("Unable to initialise pigpio (is pigpiod running?)");
    }
    ready_ = true;
}

PigpioManager::~PigpioManager() {
    if (ready_) {
        gpioTerminate();
        ready_ = false;
    }
}

HCSR04Sensor::HCSR04Sensor(int trigger_pin, int echo_pin, unsigned timeout_us)
    : trigger_pin_(trigger_pin), echo_pin_(echo_pin), timeout_us_(timeout_us) {
    gpioSetMode(trigger_pin_, PI_OUTPUT);
    gpioSetMode(echo_pin_, PI_INPUT);
    gpioWrite(trigger_pin_, PI_LOW);
}

double HCSR04Sensor::read() {
    gpioTrigger(trigger_pin_, 10, PI_HIGH);
    const uint32_t wait_start = gpioTick();
    while (gpioRead(echo_pin_) == PI_LOW) {
        if (tickDiff(wait_start, gpioTick()) > timeout_us_) {
            return last_good_reading_cm_;
        }
        gpioDelay(2);
    }

    const uint32_t pulse_start = gpioTick();
    while (gpioRead(echo_pin_) == PI_HIGH) {
        if (tickDiff(pulse_start, gpioTick()) > timeout_us_) {
            return last_good_reading_cm_;
        }
        gpioDelay(2);
    }
    const uint32_t pulse_end = gpioTick();
    const double pulse_length_us = static_cast<double>(tickDiff(pulse_start, pulse_end));
    const double distance_cm = pulse_length_us / 58.0;  // 17.2 us per cm for round trip
    last_good_reading_cm_ = distance_cm;
    return distance_cm;
}

ServoActuator::ServoActuator(int pwm_pin, unsigned min_pulse_us, unsigned max_pulse_us)
    : pwm_pin_(pwm_pin), min_pulse_us_(min_pulse_us), max_pulse_us_(max_pulse_us) {
    gpioSetMode(pwm_pin_, PI_OUTPUT);
    gpioServo(pwm_pin_, min_pulse_us_);
}

void ServoActuator::apply(double normalized_command) {
    const double clamped = std::clamp(normalized_command, 0.0, 1.0);
    const unsigned pulse = static_cast<unsigned>(min_pulse_us_ + (clamped * (max_pulse_us_ - min_pulse_us_)));
    gpioServo(pwm_pin_, pulse);
}

#endif  // ENABLE_PIGPIO
