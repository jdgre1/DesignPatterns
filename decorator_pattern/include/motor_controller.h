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
    double speed_{0.0};

public:
   
    BasicMotorController() = default;

    void SetSpeed(double speed) override {
        speed_ = speed;
        std::cout << "Speed set to " << speed << "\n";
    }

    double GetSpeed() const override {
        return speed_;
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
