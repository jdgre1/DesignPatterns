#include <controller.h>

namespace patterns
{
Controller* Controller::instancePtr = NULL;  


Controller* Controller::GetInstance()
{
    if (instancePtr == NULL) {
        instancePtr = new Controller();
    }
    return instancePtr;
}

Controller::Controller() 
{

}

void Controller::SetPlayerId(Entity e)
{
    m_player = Player::GetInstance();
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
