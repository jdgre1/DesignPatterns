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

SDL_Point MovementSystem::getNewStartingPoint(int condition, double x, double y, double angleRadians)
{
    int32_t fieldHeight = static_cast<int32_t>(FIELD_HEIGHT_MAX);
    int32_t fieldWidth = static_cast<int32_t>(FIELD_WIDTH_MAX);

    SDL_Point newPoint;

    if(condition==0)
    {
        double newX = 1.0;
        double newY = fieldHeight--;
        // SDL_Color debugColour = {0, 0, 255};

        while(newX > 0 && newY > 0)
        {
            newX = (newY - y) / tan(angleRadians + M_PI) + x;
            // std::cout << "\nNewPoint: {" << newX << ", " << newY << "}";
            // Utils::drawCircle(m_renderer, newY, newX, 5, debugColour);
            // SDL_RenderPresent(m_renderer);
            // SDL_Delay(10);
            newPoint.x = newX;
            newPoint.y = newY;
            newY--;
        }
    }

    // entityXY.posY < 1
    // int32_t newYPos = fieldHeight;
    else if(condition==1)
    {
        double newX = 1.0;
        double newY = fieldHeight--;
        newY = 0.01;
        // SDL_Color debugColour = {0, 0, 255};

        while(newX < fieldWidth && newY < fieldHeight && newX > 0 && newY > 0)
        {
            newX = (newY - y) / tan(angleRadians - M_PI) + x;
            // std::cout << "\nNewPoint: {" << newX << ", " << newY << "}";
            // Utils::drawCircle(m_renderer, newY, newX, 5, debugColour);
            // SDL_RenderPresent(m_renderer);
            // SDL_Delay(10);
            newPoint.x = newX;
            newPoint.y = newY;
            newY++;
        }

        int a = 3;
    }
    // (entityXY.posX > fieldWidth + 25) 
    else if(condition==2)
    {
        double newX = fieldWidth--;
        double newY = 1;
        newY = 0.01;
        // SDL_Color debugColour = {0, 0, 255};

        while(newX > 0 && newY > 0)
        {
            newY = (newX-x) * tan(angleRadians - M_PI) + y;
            // std::cout << "\nNewPoint: {" << newX << ", " << newY << "}";
            // Utils::drawCircle(m_renderer, newY, newX, 5, debugColour);
            // SDL_RenderPresent(m_renderer);
            // SDL_Delay(10);
            newPoint.x = newX;
            newPoint.y = newY;
            newX--;
        }

        int a = 3;
    }

     else if(condition==3)
    {
        double newX = 1;
        double newY = 1;
        newY = 0.01;
        // SDL_Color debugColour = {0, 0, 255};

        while(newX > 0 && newY > 0 && newX < fieldWidth && newY < fieldHeight)
        {
            newY = (newX-x) * tan(angleRadians + M_PI) + y;
            // std::cout << "\nNewPoint: {" << newX << ", " << newY << "}";
            // Utils::drawCircle(m_renderer, newY, newX, 5, debugColour);
            // SDL_RenderPresent(m_renderer);
            // SDL_Delay(10);
            newPoint.x = newX;
            newPoint.y = newY;
            newX++;
        }

        int a = 3;
    }


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
        double changePosX = speed * cos(angleRad);
        double changePosY = speed * sin(angleRad);
        entityXY.posX += changePosX;
        entityXY.posY += changePosY;

        auto& entityVel = compRegister.entityVelocityComponents[e];
        entityVel.velX += speed * cos(angleRad);
        entityVel.velY += speed * sin(angleRad);

        // Allow player to transition across field and begin on the other side:
        int32_t fieldHeight = static_cast<int32_t>(FIELD_HEIGHT_MAX);
        int32_t fieldWidth = static_cast<int32_t>(FIELD_WIDTH_MAX);

        if (entityXY.posY > fieldHeight + 25) {
            SDL_Point newPos = getNewStartingPoint(0, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));

        } else if (entityXY.posY < -25) {
            SDL_Point newPos = getNewStartingPoint(1, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));
        }

        else if (entityXY.posX > fieldWidth + 25) {
            SDL_Point newPos = getNewStartingPoint(2, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));
        } else if (entityXY.posX < 0) {
            SDL_Point newPos = getNewStartingPoint(3, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));
        }
    }
}

} // namespace patterns