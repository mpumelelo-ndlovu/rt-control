#include "controller.hpp"
#include "devices.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_should_stop{false};

void handleSignal(int) {
    g_should_stop.store(true);
}

struct Options {
    std::string mode = "sim";
    ControlConfig config;
    int trig_pin = 23;
    int echo_pin = 24;
    int servo_pin = 18;
    unsigned servo_min = 1000;
    unsigned servo_max = 2000;
    std::string log_file;
    int telemetry_period_ms = 500;
};

void printUsage() {
    std::cout << "Usage: rt_control [--mode sim|gpio] [--setpoint cm] [--kp value] [--ki value]\n"
                 "                 [--kd value] [--loop-hz hz] [--priority N]\n"
                 "                 [--filter-alpha 0-1] [--trig pin] [--echo pin] [--servo pin]\n"
                 "                 [--servo-min us] [--servo-max us] [--log-file path]\n"
                 "                 [--telemetry-ms interval]\n";
}

bool parseDouble(const char* value, double& out) {
    try {
        out = std::stod(value);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseUnsigned(const char* value, unsigned& out) {
    try {
        const auto parsed = std::stoul(value);
        out = static_cast<unsigned>(parsed);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseInt(const char* value, int& out) {
    try {
        out = std::stoi(value);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto requireValue = [&](auto&& parser, auto& target) {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + arg);
            }
            if (!parser(argv[++i], target)) {
                throw std::runtime_error("Invalid value supplied for " + arg);
            }
        };

        if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else if (arg == "--mode") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for --mode");
            }
            options.mode = argv[++i];
        } else if (arg == "--setpoint") {
            requireValue(parseDouble, options.config.setpoint_cm);
        } else if (arg == "--kp") {
            requireValue(parseDouble, options.config.kp);
        } else if (arg == "--ki") {
            requireValue(parseDouble, options.config.ki);
        } else if (arg == "--kd") {
            requireValue(parseDouble, options.config.kd);
        } else if (arg == "--loop-hz") {
            requireValue(parseDouble, options.config.loop_hz);
        } else if (arg == "--priority") {
            requireValue(parseInt, options.config.realtime_priority);
        } else if (arg == "--filter-alpha") {
            requireValue(parseDouble, options.config.measurement_filter_alpha);
        } else if (arg == "--feedforward") {
            requireValue(parseDouble, options.config.feedforward);
        } else if (arg == "--trig") {
            requireValue(parseInt, options.trig_pin);
        } else if (arg == "--echo") {
            requireValue(parseInt, options.echo_pin);
        } else if (arg == "--servo") {
            requireValue(parseInt, options.servo_pin);
        } else if (arg == "--servo-min") {
            requireValue(parseUnsigned, options.servo_min);
        } else if (arg == "--servo-max") {
            requireValue(parseUnsigned, options.servo_max);
        } else if (arg == "--log-file") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for --log-file");
            }
            options.log_file = argv[++i];
        } else if (arg == "--telemetry-ms") {
            requireValue(parseInt, options.telemetry_period_ms);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        options = parseArgs(argc, argv);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        printUsage();
        return 1;
    }

    if (options.telemetry_period_ms < 10) {
        options.telemetry_period_ms = 10;
    }
    if (options.config.measurement_filter_alpha < 0.0 || options.config.measurement_filter_alpha >= 1.0) {
        options.config.measurement_filter_alpha = 0.0;
    }

    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);

    std::unique_ptr<ISensor> sensor;
    std::unique_ptr<IActuator> actuator;

    if (options.mode == "sim") {
        sensor = std::make_unique<SimulatedSensor>();
        actuator = std::make_unique<SimulatedActuator>();
        options.config.realtime_priority = 0;  // avoid requesting RT scheduling on dev boxes
    } else if (options.mode == "gpio") {
#if defined(ENABLE_PIGPIO)
        try {
            static PigpioManager pigpio_guard;
            sensor = std::make_unique<HCSR04Sensor>(options.trig_pin, options.echo_pin);
            actuator = std::make_unique<ServoActuator>(options.servo_pin, options.servo_min, options.servo_max);
        } catch (const std::exception& ex) {
            std::cerr << "Failed to initialize pigpio: " << ex.what() << "\n";
            return 2;
        }
#else
        std::cerr << "Binary built without pigpio support. Reconfigure with -DENABLE_PIGPIO=ON\n";
        return 2;
#endif
    } else {
        std::cerr << "Unsupported mode: " << options.mode << "\n";
        return 1;
    }

    std::shared_ptr<std::ofstream> telemetry_file;
    if (!options.log_file.empty()) {
        telemetry_file = std::make_shared<std::ofstream>(options.log_file, std::ios::out | std::ios::trunc);
        if (!telemetry_file->good()) {
            std::cerr << "Unable to open log file: " << options.log_file << "\n";
            return 3;
        }
        *telemetry_file << "timestamp_us,distance_cm,command\n";
        std::cout << "Logging telemetry to " << options.log_file << "\n";
    }

    Controller controller(*sensor, *actuator, options.config);
    controller.start();

    const auto telemetry_period = std::chrono::milliseconds(std::max(10, options.telemetry_period_ms));
    std::thread status_thread([&controller, telemetry_period, telemetry_file]() {
        std::size_t flush_counter = 0;
        while (!g_should_stop.load()) {
            const double measurement = controller.latestMeasurement();
            const double command = controller.latestCommand();
            std::cout << "distance_cm=" << measurement << " command=" << command << "\n";
            if (telemetry_file && telemetry_file->good()) {
                const auto now = std::chrono::system_clock::now();
                const auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
                (*telemetry_file) << timestamp_us << "," << measurement << "," << command << "\n";
                if (++flush_counter % 10 == 0) {
                    telemetry_file->flush();
                }
            }
            std::this_thread::sleep_for(telemetry_period);
        }
        if (telemetry_file) {
            telemetry_file->flush();
        }
    });

    while (!g_should_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    controller.stop();
    if (status_thread.joinable()) {
        status_thread.join();
    }

    return 0;
}
