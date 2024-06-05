#pragma once

#ifndef PLAYER_H
#define PLAYER_H

#include <character.h>
#include <components.h>
#include <utils.h>

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>


namespace patterns {

enum KeyboardCommand
{
    NO_COMMAND,
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD,
    SPEED_INCREASE,
    SPEED_DECREASE
};

class Player : public Character
{
    public:

        void SetEntityId(Entity e){
            m_id = e;
        }
        Entity GetId(){
            return m_id;
        }

        KeyboardCommand GetLastKeyboardCommand(){return m_lastKeyboardCommand;}
        double GetAngleOfRotation(){ return m_directionThetaDegrees;}
        double SetAngleOfRotation(double angle){ return m_directionThetaDegrees = angle;}
        void SetDirection(Direction d){m_direction = d;}
        double GetSpeed(){ return m_currSpeed;}
        void SetSpeed(double speed){m_currSpeed = speed;}
        Direction GetDirection(){ return m_direction;}
        void setLastKeyboardCommand(KeyboardCommand kcmd){
            m_lastKeyboardCommand = kcmd;
        }
        void turnLeft();
        void turnRight();
        void forwards();
        void backwards();
        void incrementSpeed();
        void decrementSpeed();
        void stop();

    private:
        Components::XYComponent m_xyComponent;
        Components::VelocityComponent m_velComponent;
        Entity m_id;
        double m_directionThetaDegrees = 0.0;
        double m_currSpeed = 0.0;
        KeyboardCommand m_lastKeyboardCommand;
        Direction m_direction;

};
} // namespace patterns
#endif  // PLAYER_H
