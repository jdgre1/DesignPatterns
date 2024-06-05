#include <player.h>

namespace patterns {


void Player::turnLeft()
{
    // if(m_xyComponent.posX > 0){
    //     m_xyComponent.posX--;
    // }
    m_lastKeyboardCommand = KeyboardCommand::LEFT;

}

void Player::turnRight()
{
    // if(m_xyComponent.posX < FIELD_WIDTH_MAX){
    //     m_xyComponent.posX++;
    // }
    m_lastKeyboardCommand = KeyboardCommand::RIGHT;

}

void Player::forwards()
{
    // if(m_xyComponent.posY < FIELD_HEIGHT_MAX){
    //     m_xyComponent.posY++;
    // }
    m_lastKeyboardCommand = KeyboardCommand::FORWARD;

}

void Player::backwards()
{
    // if(m_xyComponent.posY > 0){
    //     m_xyComponent.posY--;
    // }
    m_lastKeyboardCommand = KeyboardCommand::BACKWARD;

}

void Player::incrementSpeed()
{
    // m_velComponent.velX+= 0.1;
    // m_velComponent.velY+= 0.1;
    m_lastKeyboardCommand = KeyboardCommand::SPEED_INCREASE;

}

void Player::decrementSpeed()
{
    // m_velComponent.velX-= 0.1;
    // m_velComponent.velY-= 0.1;
    m_lastKeyboardCommand = KeyboardCommand::SPEED_DECREASE;

}

void Player::stop()
{
    m_velComponent.velX = 0.0;
    m_velComponent.velY = 0.0;
}

}