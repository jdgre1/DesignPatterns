#pragma once
#include <iostream>
#include <memory>

namespace patterns
{

class MotorController
{

public:
    virtual void SetSpeed(double speed) = 0;
    virtual double GetSpeed() const = 0;
    virtual ~MotorController() = default;
};

class BasicMotorController : public MotorController
{
    std::shared_ptr<MotorController> base_;

public:
    BasicMotorController(std::shared_ptr<MotorController> base = nullptr) : base_(std::move(base))
    {
        if (!base_) {
            throw std::invalid_argument("ThermalProtectionDecorator requires a valid MotorController");
        }
    }

    void SetSpeed(double speed) override
    {
        if (base_) {
            base_->SetSpeed(speed);
        }
        else {
            std::cout << "[Basic] Speed set to " << speed << std::endl;
        }
    }
    double GetSpeed() const override
    {
        return base_ ? base_->GetSpeed() : 0.0; // Dummywert
    }

protected:
    virtual double getMotorVoltage()
    {
        return 24.0;
    }
    virtual double getMotorCurrent()
    {
        return 2.5;
    }
    virtual double getMotorSpeed()
    {
        return 1500.0;
    }
    virtual double getMotorPosition()
    {
        return 1234;
    }
    virtual double getPwmDutyCycle()
    {
        return 0.65;
    }
};

} // namespace patterns
