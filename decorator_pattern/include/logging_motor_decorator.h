#pragma once
#include <motor_controller.h>
#include <sensor_decorator.h>

namespace patterns
{

class LoggingMotorDecorator : public MotorController
{

    std::shared_ptr<MotorController> base_;

public:
    LoggingMotorDecorator(std::shared_ptr<MotorController> base) : base_(std::move(base))
    {
        if (!base_) {
            throw std::invalid_argument("LoggingMotorDecorator requires a valid MotorController");
        }
    }

    void SetSpeed(double speed) override
    {
        std::cout << "[Log] SetSpeed called with value: " << speed << std::endl;
        base_->SetSpeed(speed);
    }
    double GetSpeed() const override
    {
        return base_->GetSpeed();
    }

    void LogInternalStatus() const
    {
        if (auto sensor = std::dynamic_pointer_cast<SensorDecorator>(base_)) {
            std::cout << "[Log] Voltage: " << sensor->MotorVoltage() << " V, Current: " << sensor->MotorCurrent()
                      << " A, Speed: " << sensor->MotorRPM() << " RPM, Position: " << sensor->MotorPosition()
                      << ", PWM: " << sensor->PwmDuty() * 100 << "%" << std::endl;
        }
        else {
            std::cout << "[Log] Diagnostics not available\n";
        }
    }
};
} // namespace patterns
