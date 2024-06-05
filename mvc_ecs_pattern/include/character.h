#pragma once

#ifndef CHARACTER_H
#define CHARACTER_H

#include <components.h>
#include <utils.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <cassert>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <utils.h>
#include <ctime>

namespace patterns
{

enum Direction { FORWARDS, BACKWARDS };

class Character
{
public:
    Entity GetId(){
        return m_id;
    };
    void setColour(SDL_Color colour){
        m_characterColour = colour;
    }
    virtual int attack(){
        return rand() % 4;
    }

    void initialisePosition()
    {
        std::srand(std::time(nullptr));
    
        // Generate a random number between 0 and 800
        m_xyComponent.posX = std::rand() % FIELD_WIDTH_MAX;
        m_xyComponent.posY = std::rand() % FIELD_HEIGHT_MAX;

    }

protected:
    Entity m_id;
    SDL_Color m_characterColour = {120, 255, 120};
    Components::XYComponent m_xyComponent;
    Components::VelocityComponent m_velComponent;
    double m_directionThetaDegrees = 0.0;
    double m_currSpeed = 0.0;
    Direction m_direction;


};
#endif // CHARACTER_H

} // namespace patterns
