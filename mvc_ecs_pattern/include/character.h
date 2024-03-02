#pragma once

#ifndef CHARACTER_H
#define CHARACTER_H


#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <utils.h>


namespace patterns {

enum Direction
{
    FORWARDS,
    BACKWARDS
};

class Character
{
    public:
        virtual Entity GetId() = 0;
       

};
} // namespace patterns
#endif  // CHARACTER_H
