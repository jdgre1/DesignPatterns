#pragma once

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <player.h>

#include <cassert>
#include <iostream>
#include <stdint.h>
#include <stdio.h>

namespace patterns
{

class Controller
{
public:
    // Delete copy constructor
    Controller(const Controller& obj) = delete;
    /** * Singletons should not be assignable. */
    void operator=(const Controller&) = delete;

    static Controller* GetInstance();

    void SetPlayerId(Entity e);
    Player* GetPlayer() { return m_player; }
    void userInputLeft();
    void userInputRight();
    void userInputForwards();
    void userInputBackwards();
    void userInputIncreaseSpeed();
    void userInputDecreaseSpeed();
    void userInputStop();

private:
    Controller();

    static Controller* instancePtr;
    Player* m_player;
};
} // namespace patterns
#endif // CONTROLLER_H
