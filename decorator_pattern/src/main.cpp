// #pragma once
#include <logging_motor_decorator.h>
#include <safety_limit_decorator.h>
#include <sensor_decorator.h>
#include <thermal_protection_decorator.h>


int main() {
    
    std::shared_ptr<patterns::MotorController> motor = std::make_shared<patterns::BasicMotorController>();

    // // Wrap with safety limiter (e.g., max speed = 5.0)
    motor = std::make_shared<patterns::SafetyLimiterDecorator>(motor, 5.0);

    // // Wrap with sensor measurement capability 
    motor = std::make_shared<patterns::SensorDecorator>(motor);

    // // Wrap with thermal/overheating protection 
    motor = std::make_shared<patterns::ThermalProtectionDecorator>(motor);

    // Wrap with logging
    motor = std::make_shared<patterns::LoggingMotorDecorator>(motor);

    motor->SetSpeed(3.0);   // Should pass through both decorators
    motor->SetSpeed(8.0);   // Capped to 5.0 due to safety-limit decorator

    return 0;
}
