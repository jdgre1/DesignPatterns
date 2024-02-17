#pragma once

#ifndef PLAYER_H
#define PLAYER_H

#include <components.h>
#include <utils.h>

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>


namespace patterns {

enum KeyboardCommand
{
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD,
    SPEED_INCREASE,
    SPEED_DECREASE
};

class Player
{
    public:

        // Delete copy constructor
        Player(const Player& obj) = delete; 
        /** * Singletons should not be assignable. */
        void operator=(const Player&) = delete;
        static Player* GetInstance();

        ~Player(){}
        void SetEntityId(Entity e){
            m_id = e;
        }

        KeyboardCommand GetLastKeyboardCommand(){return m_lastKeyboardCommand;}

        void turnLeft();
        void turnRight();
        void forwards();
        void backwards();
        void incrementSpeed();
        void decrementSpeed();
        void stop();

    private:
        Player() {}
        static Player* instancePtr;
        Components::XYComponent m_xyComponent;
        Components::VelocityComponent m_velComponent;
        Entity m_id;
        double m_directionThetaDegrees = 0.0;
        KeyboardCommand m_lastKeyboardCommand;


};
} // namespace patterns
#endif  // PLAYER_H
