cmake_minimum_required(VERSION 3.10)
project(RoboticsDecoratorPattern)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(robotics_decorator_pattern
    src/main.cpp
    include/motor_controller.h
    include/logging_motor_decorator.h
    include/safety_limit_decorator.h
    include/sensor_decorator.h
    include/thermal_protection_decorator.h
)

