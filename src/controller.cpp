#include "controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <thread>

#if defined(__linux__)
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#endif

namespace {

#if defined(__linux__)
timespec chronoToTimespec(const std::chrono::steady_clock::time_point& tp) {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
    timespec ts;
    ts.tv_sec = static_cast<time_t>(ns / 1'000'000'000LL);
    ts.tv_nsec = static_cast<long>(ns % 1'000'000'000LL);
    return ts;
}

void sleepUntilLinux(const timespec& ts) {
    timespec deadline = ts;
    while (true) {
        const int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
        if (rc == 0) {
            break;
        }
        if (rc != EINTR) {
            break;
        }
    }
}
#endif

}  // namespace

Controller::Controller(ISensor& sensor, IActuator& actuator, ControlConfig config)
    : sensor_(sensor), actuator_(actuator), config_(config) {}

Controller::~Controller() {
    stop();
}

void Controller::start() {
    if (running_.load()) {
        return;
    }
    stop_requested_.store(false);
    running_.store(true);
    worker_ = std::thread(&Controller::loop, this);
}

void Controller::stop() {
    stop_requested_.store(true);
    if (worker_.joinable()) {
        worker_.join();
    }
    running_.store(false);
}

double Controller::computePid(double measurement, double dt_seconds) {
    const double error = config_.setpoint_cm - measurement;
    if (dt_seconds <= 0.0) {
        dt_seconds = 1.0 / config_.loop_hz;
    }

    integral_state_ += error * dt_seconds;
    integral_state_ = std::clamp(integral_state_, -config_.max_integral, config_.max_integral);

    const double derivative = (error - previous_error_) / dt_seconds;
    previous_error_ = error;

    const double output = config_.feedforward + (config_.kp * error) + (config_.ki * integral_state_) + (config_.kd * derivative);
    const double clamped = std::clamp(output, config_.actuator_min, config_.actuator_max);
    const double denom = config_.actuator_max - config_.actuator_min;
    if (denom <= 0.0) {
        return config_.actuator_min;
    }
    return (clamped - config_.actuator_min) / denom;
}

void Controller::loop() {
#if defined(__linux__)
    if (config_.realtime_priority > 0) {
        sched_param param{};
        param.sched_priority = config_.realtime_priority;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    }
#endif

    const auto period = std::chrono::duration<double>(1.0 / config_.loop_hz);
    auto next_deadline = std::chrono::steady_clock::now() + period;
    auto previous_sample_time = std::chrono::steady_clock::now();

    while (!stop_requested_.load()) {
        const double measurement = sensor_.read();
        last_measurement_.store(measurement, std::memory_order_relaxed);

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - previous_sample_time).count();
        previous_sample_time = now;

        const double command = computePid(measurement, dt);
        last_command_.store(command, std::memory_order_relaxed);
        actuator_.apply(command);

        next_deadline += period;
#if defined(__linux__)
        sleepUntilLinux(chronoToTimespec(next_deadline));
#else
        std::this_thread::sleep_until(next_deadline);
#endif
    }
}
