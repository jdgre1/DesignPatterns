#pragma once

#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <vector>

namespace patterns {

class Components
{
    public:
        struct XYComponent
        {   
            void setPosition(int32_t x, int32_t y)
            {
                posX = x;
                posY = x;
            }
            int32_t posX;
            int32_t posY;
        };

        struct VelocityComponent
        {   
            void setVelocity(float x, float y)
            {
                velX = x;
                velY = y;
            }
            float velX;
            float velY;
        };

        Components();
        ~Components(){}
  

    private:
        XYComponent m_xyComponent;
        VelocityComponent m_velComponent;


};
} // namespace patterns

#endif  // COMPONENTS_H