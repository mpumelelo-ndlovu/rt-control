#include "devices.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

SimulatedSensor::SimulatedSensor()
    : start_time_(std::chrono::steady_clock::now()),
      rng_(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count())) {}

double SimulatedSensor::read() {
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    const double oscillation = 35.0 + (10.0 * std::sin(elapsed * 0.6));
    const double signal = oscillation + noise_(rng_);
    return std::clamp(signal, 5.0, 80.0);
}

void SimulatedActuator::apply(double normalized_command) {
    last_command_ = std::clamp(normalized_command, 0.0, 1.0);
}
