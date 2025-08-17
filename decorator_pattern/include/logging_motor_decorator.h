#pragma once
#include <motor_controller.h>


namespace patterns {

class LoggingMotorDecorator : public MotorController 
{

    std::shared_ptr<MotorController> base_;

    public:
        LoggingMotorDecorator(std::shared_ptr<MotorController> base) 
        : base_(std::move(base)) {}

        void SetSpeed(double speed) override 
        {
            std::cout << "[Log] SetSpeed called with value: " << speed << std::endl;
            base_->SetSpeed(speed);
        }

};
}
