#include "controller.hpp"
#include "devices.hpp"

#include <atomic>
#include <cstdlib>
#include <csignal>
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
};

void printUsage() {
    std::cout << "Usage: rt_control [--mode sim|gpio] [--setpoint cm] [--kp value] [--ki value]\n"
                 "                 [--kd value] [--loop-hz hz] [--priority N]\n"
                 "                 [--trig pin] [--echo pin] [--servo pin]\n"
                 "                 [--servo-min us] [--servo-max us]\n";
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

    Controller controller(*sensor, *actuator, options.config);
    controller.start();

    std::thread status_thread([&controller]() {
        using namespace std::chrono_literals;
        while (!g_should_stop.load()) {
            std::cout << "distance_cm=" << controller.latestMeasurement()
                      << " command=" << controller.latestCommand() << "\n";
            std::this_thread::sleep_for(500ms);
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
