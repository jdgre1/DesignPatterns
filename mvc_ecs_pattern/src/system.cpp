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

SDL_Point MovementSystem::getNewStartingPoint(double x, double y, double angleRadians)
{
    int32_t fieldHeight = static_cast<int32_t>(FIELD_HEIGHT_MAX);
    int32_t fieldWidth = static_cast<int32_t>(FIELD_WIDTH_MAX);
    double newX = 1.0;
    double newY = fieldHeight--;
    SDL_Color debugColour = {0, 0, 255};
    while(newX > 0 && newY > 0)
    {   
        newX = (newY - y) / tan(angleRadians + M_PI) + x;
        std::cout << "\nNewPoint: {" << newX << ", " << newY << "}";
        Utils::drawCircle(m_renderer, newY, newX, 5, debugColour);
        SDL_RenderPresent(m_renderer);
        SDL_Delay(10);
        newY--;
    }
    SDL_Point newPoint;
    newPoint.x = newX;
    newPoint.y = newY;
    return newPoint;
}



void MovementSystem::update()
{
    EntityManager* entityManager = EntityManager::GetInstance();
    EntityManager::ComponentRegister& compRegister = entityManager->GetComponentRegister();

    // 1. Loop over each entity
    std::unordered_map<Entity, Player*>& entityPlayerRegister = entityManager->GetEntityPlayerRegister();
    for (auto& it : entityPlayerRegister) {
        Entity e = it.first;
        Player* p = it.second;

        KeyboardCommand k = p->GetLastKeyboardCommand();
        double speed = p->GetSpeed();
        double angle = p->GetAngleOfRotation();
        Direction d = p->GetDirection();

        // TODO
        // if going forwards and backwards was hit previously need to skip switch statement
        //  and allow speed to keep decreasing for several engine ticks
        switch (k) {
        case KeyboardCommand::LEFT: {
            angle += m_angleIncrement;
            p->SetAngleOfRotation(angle);
            if (angle >= 360.0) {
                angle = 0;
            }
            p->SetAngleOfRotation(angle);
        }

        break;
        case KeyboardCommand::RIGHT: {
            angle -= m_angleIncrement;
            if (angle < 0.0) {
                angle = 359;
            }
            p->SetAngleOfRotation(angle);

        } break;
        case KeyboardCommand::FORWARD: {
            p->SetDirection(Direction::FORWARDS);

        } break;
        case KeyboardCommand::BACKWARD: {
            p->SetDirection(Direction::BACKWARDS);

        } break;

            break;
        case KeyboardCommand::SPEED_INCREASE: {
            speed++;
            p->SetSpeed(speed);

        } break;
        case KeyboardCommand::SPEED_DECREASE: {
            speed--;
            p->SetSpeed(speed);

        } break;

        default:
            break;
        }
        p->setLastKeyboardCommand(KeyboardCommand::NO_COMMAND);
        // Update components for one tick of the engine:
        auto& entityXY = compRegister.entityXYComponents[e];
        double t = 0.01; // 10ms engine tick
        double angleRad = angle * M_PI / 180.0;
        std::cout << "\nangleDeg = " << angle;
        std::cout << "\nangleRad = " << angleRad;
        double changePosX = speed * cos(angleRad);
        double changePosY = speed * sin(angleRad);
        entityXY.posX += changePosX;
        entityXY.posY += changePosY;

        auto& entityVel = compRegister.entityVelocityComponents[e];
        entityVel.velX += speed * cos(angleRad);
        entityVel.velY += speed * sin(angleRad);

        // Allow player to transition across field and begin on the other side:
        if(angleRad < 0.0001) {return;}
        int32_t fieldHeight = static_cast<int32_t>(FIELD_HEIGHT_MAX);
        int32_t fieldWidth = static_cast<int32_t>(FIELD_WIDTH_MAX);

        if (entityXY.posY > fieldHeight + 25) {

            SDL_Point newPos = getNewStartingPoint(entityXY.posX, entityXY.posY, angleRad);
            int32_t newYPos = 0;
            double newXPos = (newYPos - entityXY.posY) / tan(angleRad + M_PI) + entityXY.posX ;
            // double newXPos = entityXY.posX + (newYPos- entityXY.posY ) / tan(angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));

        } else if (entityXY.posY < 1) {
            int32_t newYPos = fieldHeight;
            // double newXPos = entityXY.posX + (newYPos- entityXY.posY ) / tan(angleRad);
            // double newXPos = -tan(angleRad) * (entityXY.posY - newYPos) / entityXY.posX ;
            // entityXY.posX = static_cast<int32_t>(std::round(newXPos));
            entityXY.posY = newYPos;
        }

        if (entityXY.posX > fieldWidth + 25) {
            int32_t newXPos = 0;
            // double newYPos = entityXY.posX + (newYPos- entityXY.posY ) / tan(angleRad);
            // entityXY.posY = static_cast<int32_t>(std::round(newYPos));
            entityXY.posX = newXPos;
        } else if (entityXY.posX < 1) {
            int32_t newXPos = fieldWidth;
            // double newYPos = entityXY.posX + (newYPos- entityXY.posY ) / tan(angleRad);
            // entityXY.posY = static_cast<int32_t>(std::round(newYPos));
            entityXY.posX = newXPos;
        }
    }
}

} // namespace patterns