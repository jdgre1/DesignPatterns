#pragma once
#include <iostream>
#include <memory>

namespace patterns {

class MotorController {

    public:
        virtual void SetSpeed(double speed) = 0;
        virtual ~MotorController() = default;
};

class BasicMotorController : public MotorController 
{

    public:
        void SetSpeed(double speed) override 
        {
            std::cout << "[Motor] Speed set to: " << speed << std::endl;
        }
};

}