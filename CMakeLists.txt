cmake_minimum_required(VERSION 3.16)
project(rt_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_PIGPIO "Enable pigpio GPIO support" ON)

add_executable(rt_control
    src/main.cpp
    src/controller.cpp
    src/simulated_devices.cpp
)

target_include_directories(rt_control PRIVATE include)
target_compile_options(rt_control PRIVATE -Wall -Wextra -Wpedantic)

if (ENABLE_PIGPIO)
    find_path(PIGPIO_INCLUDE_DIR pigpio.h)
    find_library(PIGPIO_LIBRARY pigpio)
    if (PIGPIO_INCLUDE_DIR AND PIGPIO_LIBRARY)
        target_sources(rt_control PRIVATE src/gpio_devices.cpp)
        target_include_directories(rt_control PRIVATE ${PIGPIO_INCLUDE_DIR})
        target_link_libraries(rt_control PRIVATE ${PIGPIO_LIBRARY})
        target_compile_definitions(rt_control PRIVATE ENABLE_PIGPIO=1)
    else()
        message(WARNING "pigpio headers or library not found; GPIO mode disabled")
    endif()
endif()

if (UNIX)
    target_link_libraries(rt_control PRIVATE pthread)
endif()
