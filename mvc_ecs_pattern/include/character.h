#pragma once

#ifndef CHARACTER_H
#define CHARACTER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <cassert>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <utils.h>

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

protected:
    Entity m_id;
    SDL_Color m_characterColour = {120, 255, 120};
};
#endif // CHARACTER_H

} // namespace patterns
