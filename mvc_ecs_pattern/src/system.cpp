#include <entity_manager.h>
#include <system.h>

#include <cmath>
namespace patterns
{


MovementSystem& MovementSystem::GetInstance(patterns::EntityManager& entityMngr)
{
    static MovementSystem ms(entityMngr);
    return ms;
}

MovementSystem::MovementSystem(EntityManager& entityMngr) 
    : m_em(entityMngr)
{}


SDL_Point MovementSystem::getNewStartingPoint(NewStartingPosition pos, double x, double y, double angleRadians)
{
    int32_t fieldHeight = static_cast<int32_t>(FIELD_HEIGHT_MAX);
    int32_t fieldWidth = static_cast<int32_t>(FIELD_WIDTH_MAX);

    SDL_Point newPoint;

    switch(pos) {
    case NewStartingPosition::TOP:
    {
        double newX = 1.0;
        double newY = fieldHeight--;

        while(newX > 0 && newY > 0)
        {
            newX = (newY - y) / tan(angleRadians + M_PI) + x;
            newPoint.x = newX;
            newPoint.y = newY;
            newY--;
        }
        break;
    }

    case NewStartingPosition::BOTTOM:
    {
        double newX = 1.0;
        double newY = fieldHeight--;
        newY = 0.01;

        while(newX < fieldWidth && newY < fieldHeight && newX > 0 && newY > 0)
        {
            newX = (newY - y) / tan(angleRadians - M_PI) + x;
            newPoint.x = newX;
            newPoint.y = newY;
            newY++;
        }
        break;
    }
    case NewStartingPosition::RIGHT_SIDE:
    {
        double newX = fieldWidth--;
        double newY = 1;
        newY = 0.01;
        while(newX > 0 && newY > 0)
        {
            newY = (newX-x) * tan(angleRadians - M_PI) + y;
            newPoint.x = newX;
            newPoint.y = newY;
            newX--;
        }

        break;
    }

    case NewStartingPosition::LEFT_SIDE:
    {
        double newX = 1;
        double newY = 1;
        newY = 0.01;

        while(newX > 0 && newY > 0 && newX < fieldWidth && newY < fieldHeight)
        {
            newY = (newX-x) * tan(angleRadians + M_PI) + y;
            newPoint.x = newX;
            newPoint.y = newY;
            newX++;
        }

        break;
    }

    default:
        break;
    }
    return newPoint;
}



void MovementSystem::update()
{
    EntityManager::ComponentRegister& compRegister = m_em.GetComponentRegister();

    // 1. Loop over each entity
    std::unordered_map<Entity, std::shared_ptr<Player>>& entityPlayerRegister = m_em.GetEntityPlayerRegister();
    for (auto& it : entityPlayerRegister) {
        Entity e = it.first;
        std::shared_ptr<Player> p = it.second;

        KeyboardCommand k = p->GetLastKeyboardCommand();
        double speed = p->GetSpeed();
        double angle = p->GetAngleOfRotation();
        Direction d = p->GetDirection();
        // TODO
        // if going forwards and backwards was hit previously need to skip switch statement
        //  and allow speed to keep decreasing for several engine ticks
        switch (k) {
        case KeyboardCommand::LEFT: {
            angle -= m_angleIncrement;
        }

        break;
        case KeyboardCommand::RIGHT: {
            angle += m_angleIncrement;
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
        if (angle >= 360.0) {
                angle = 0;
        }
        else if (angle < 0.0) {
            angle = 359;
        }
        p->SetAngleOfRotation(angle);


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
            SDL_Point newPos = getNewStartingPoint(NewStartingPosition::TOP, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));

        } else if (entityXY.posY < -25) {
            SDL_Point newPos = getNewStartingPoint(NewStartingPosition::BOTTOM, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));
        }
        else if (entityXY.posX > fieldWidth + 25) {
            SDL_Point newPos = getNewStartingPoint(NewStartingPosition::RIGHT_SIDE, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));
        } else if (entityXY.posX < -25) {
            SDL_Point newPos = getNewStartingPoint(NewStartingPosition::LEFT_SIDE, entityXY.posX, entityXY.posY, angleRad);
            entityXY.posX = static_cast<int32_t>(std::round(newPos.x));
            entityXY.posY = static_cast<int32_t>(std::round(newPos.y));
        }
    }
}

} // namespace patterns