#include <motor_controller.h>

namespace patterns {

class ThermalProtectionDecorator : BasicMotorController
{
    std::shared_ptr<BasicMotorController> base_;
    
    public:
        ThermalProtectionDecorator(std::shared_ptr<BasicMotorController> base)
        :  base_(std::move(base))
        {}

    void SetSpeed(double speed) override
    {
        base_->SetSpeed(speed);
        std::cout << "ThermalProtectionDecorator SetSpeed called with speed" << speed << "." << std::endl;
    }
};
}