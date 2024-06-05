#include <controller.h>

namespace patterns
{

Controller& Controller::GetInstance(std::shared_ptr<Player> plyr)
{
    static Controller controller(plyr);
    return controller;
}

Controller::Controller(std::shared_ptr<Player> plyr) 
: m_player(plyr)

{

}

void Controller::SetPlayerId(Entity e)
{
    m_player->SetEntityId(e);
}

void Controller::userInputLeft()
{
    m_player->turnLeft();
}

void Controller::userInputRight()
{
    m_player->turnRight();
}

void Controller::userInputForwards()
{
    m_player->forwards();
}

void Controller::userInputBackwards()
{
    m_player->backwards();
}

void Controller::userInputIncreaseSpeed()
{
    m_player->incrementSpeed();
}

void Controller::userInputDecreaseSpeed()
{
    m_player->decrementSpeed();
}

void Controller::userInputStop()
{
    m_player->stop();
}


} // namespace patterns
