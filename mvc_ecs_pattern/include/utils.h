#pragma once

#ifndef UTILS_H
#define UTILS_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <stdint.h>
#include <stdio.h>

namespace patterns
{

extern size_t FIELD_HEIGHT_MAX;
extern size_t FIELD_WIDTH_MAX;
using Entity = uint32_t; // An entity is just an id

const Entity MAX_NUM_ENTITIES = 100;

class Utils
{
public:
    static void drawCircle(SDL_Renderer* renderer, int x, int y, int radius, SDL_Color color);
    static void drawCircle(SDL_Renderer * renderer, int centreX, int centreY, int radius);

};


} // namespace patterns

#endif // UTILS_H
