#pragma once
#include <motor_controller.h>
#include <sensor_decorator.h>

namespace patterns
{

class ThermalProtectionDecorator : public MotorController
{
    std::shared_ptr<MotorController> base_;
    double maxSafeTemperature_ = 80.0; // °C

public:
    explicit ThermalProtectionDecorator(std::shared_ptr<MotorController> base) : base_(std::move(base))
    {
        if (!base_) {
            throw std::invalid_argument("ThermalProtectionDecorator requires a valid MotorController");
        }
    }

    double GetSpeed() const override
    {
        return base_->GetSpeed();
    }

    void SetSpeed(double speed) override
    {
        // Try to get sensor interface
        auto sensor = std::dynamic_pointer_cast<SensorDecorator>(base_);
        if (sensor && sensor->MotorTemperature() > maxSafeTemperature_) {
            std::cout << "[Thermal] WARNING: Motor overheating (" << sensor->MotorTemperature()
                      << " °C)! Speed limited." << std::endl;
            base_->SetSpeed(0.0); // or clamp down
        }
        else {
            base_->SetSpeed(speed);
        }
    }
};
} // namespace patterns