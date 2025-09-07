#pragma once
#include <motor_controller.h>

namespace patterns
{

class SensorDecorator : public MotorController
{
    std::shared_ptr<MotorController> base_;

public:
    explicit SensorDecorator(std::shared_ptr<MotorController> base) : base_(std::move(base))
    {
        if (!base_) {
            throw std::invalid_argument("SensorDecorator requires a valid MotorController");
        }
    }

    void SetSpeed(double speed) override
    {
        base_->SetSpeed(speed);
    }
    double GetSpeed() const override
    {
        return base_ ? base_->GetSpeed() : 0.0; // Dummywert
    }

    // New diagnostic interface
    virtual double MotorVoltage() const
    {
        return 24.0;
    }
    virtual double MotorCurrent() const
    {
        return 2.5;
    }
    virtual double MotorRPM() const
    {
        return base_->GetSpeed();
    }
    virtual double MotorPosition() const
    {
        return 1234;
    }
    virtual double PwmDuty() const
    {
        return 0.65;
    }
    virtual double MotorTemperature() const
    {
        return 65.0;
    }
};

}; // namespace patterns