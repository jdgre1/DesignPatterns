#include <motor_controller.h>

namespace patterns {


class SafetyLimiterDecorator : public MotorController 
{

    std::shared_ptr<MotorController> base_;
    double maxSpeed_;

    public:
        SafetyLimiterDecorator(std::shared_ptr<MotorController> base, double maxSpeed)
            : base_(std::move(base)), maxSpeed_(maxSpeed) {}

        void SetSpeed(double speed) override 
        {
            if (speed > maxSpeed_) {
                std::cout << "[Safety] Speed capped from " << speed << " to " << maxSpeed_ << std::endl;
                speed = maxSpeed_;
            }
            base_->SetSpeed(speed);
        }
    };
}