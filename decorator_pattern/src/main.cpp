#include <motor_controller.h>

int main() {
    std::shared_ptr<patterns::MotorController> motor = std::make_shared<patterns::BasicMotorController>();

    // Wrap with logging
    motor = std::make_shared<patterns::LoggingMotorDecorator>(motor);

    // Wrap with safety limiter (e.g., max speed = 5.0)
    motor = std::make_shared<patterns::SafetyLimiterDecorator>(motor, 5.0);

    // Wrap with thermal/overheating protection (e.g., max speed = 5.0)
    motor = std::make_shared<patterns::ThermalProtectionDecorator>(motor);

    motor->SetSpeed(3.0);   // Should pass through both decorators
    motor->SetSpeed(8.0);   // Capped to 5.0 due to safety-limit decorator

    return 0;
}
