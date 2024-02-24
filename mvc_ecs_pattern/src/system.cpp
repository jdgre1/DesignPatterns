#include <entity_manager.h>
#include <system.h>

#include <cmath>
namespace patterns
{

MovementSystem* MovementSystem::instancePtr = NULL;

MovementSystem* MovementSystem::GetInstance()
{
    if (instancePtr == NULL) {
        instancePtr = new MovementSystem();
    }
    return instancePtr;
}
void MovementSystem::update()
{
    EntityManager* entityManager = EntityManager::GetInstance();
    EntityManager::ComponentRegister& compRegister = entityManager->GetComponentRegister();

    // 1. Loop over each entity
    std::unordered_map<Entity, Player*>& entityPlayerRegister = entityManager->GetEntityPlayerRegister();
    size_t count = 0;
    for (auto& it : entityPlayerRegister) {
        Entity e = it.first;
        Player* p = it.second;

        KeyboardCommand k = p->GetLastKeyboardCommand();
        double speedOld = p->GetSpeed();
        double angleOld = p->GetAngleOfRotation();
        Direction d = p->GetDirection();

        // TODO
        // if going forwards and backwards was hit previously need to skip switch statement
        //  and allow speed to keep decreasing for several engine ticks
        switch (k) {
        case KeyboardCommand::LEFT: {
            std::cout << "left";
            double newAngle = angleOld + m_angleIncrement;
            p->SetAngleOfRotation(newAngle);
            if (newAngle >= 360.0) {
                newAngle = 0;
            }
            p->SetAngleOfRotation(newAngle);
        }

        break;
        case KeyboardCommand::RIGHT: {
            std::cout << "right";
            double newAngle = angleOld - m_angleIncrement;
            if (newAngle < 0.0) {
                newAngle = 359;
            }
            p->SetAngleOfRotation(newAngle);

        } break;
        case KeyboardCommand::FORWARD: {
            p->SetDirection(Direction::FORWARDS);
            std::cout << "forward";

        } break;
        case KeyboardCommand::BACKWARD: {
            std::cout << "backward";
            p->SetDirection(Direction::BACKWARDS);

        } break;

            break;
        case KeyboardCommand::SPEED_INCREASE: {
            std::cout << "increase-speed";
            double currentSpeed = speedOld;
            currentSpeed++;
            p->SetSpeed(currentSpeed);

        } break;
        case KeyboardCommand::SPEED_DECREASE: {
            std::cout << "decrease-speed";
            double currentSpeed = speedOld;
            currentSpeed--;
            p->SetSpeed(currentSpeed);

        } break;

        default:
            break;
        }
        p->setLastKeyboardCommand(KeyboardCommand::NO_COMMAND);
        // std::cout << "count:" << count++;
        // Update components for one tick of the engine:
        auto& entityXY = compRegister.entityXYComponents[e];
        double t = 0.01; // 10ms engine tick
        double changePosX = speedOld * 1 * cos(angleOld * M_PI / 180);
        double changePosY = speedOld * 1 * sin(angleOld * M_PI / 180);
        entityXY.posX += changePosX;
        entityXY.posY += changePosY;

        auto& entityVel = compRegister.entityVelocityComponents[e];
        entityVel.velX += speedOld * cos(angleOld * M_PI / 180);
        entityVel.velY += speedOld * sin(angleOld * M_PI / 180);

        // Prevent going outside boundary:
        if(entityXY.posY > FIELD_HEIGHT_MAX+25){
            entityXY.posY = FIELD_HEIGHT_MAX;
        }

        if(entityXY.posX > FIELD_WIDTH_MAX+25){
            entityXY.posX = FIELD_WIDTH_MAX;
        }
    }
}

} // namespace patterns