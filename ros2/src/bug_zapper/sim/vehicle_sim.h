#pragma once
#ifndef VEHICLE_SIM_H
#define VEHICLE_SIM_H

#include <components.h>
#include <cmath>

namespace patterns {

class VehicleSim
{
public:
    VehicleSim() 
        : m_xyComponent{0.0, 0.0}, m_velocityX(0.0), m_velocityY(0.0)
    {}

    // Update the vehicle's position based on its velocity and time step
    void updatePosition(double deltaTime)
    {
        m_xyComponent.posX += m_velocityX * deltaTime;
        m_xyComponent.posY += m_velocityY * deltaTime;
    }

    // Set velocity of the vehicle in x and y directions
    void setVelocity(double velocityX, double velocityY)
    {
        m_velocityX = velocityX;
        m_velocityY = velocityY;
    }

    // Getters for velocity components
    double getVelocityX() const { return m_velocityX; }
    double getVelocityY() const { return m_velocityY; }

    // Getters for position components
    double getPositionX() const { return m_xyComponent.posX; }
    double getPositionY() const { return m_xyComponent.posY; }

private:
    struct XYComponent
    {
        double posX;
        double posY;
    } m_xyComponent;

    double m_velocityX;
    double m_velocityY;
};

} // namespace patterns

#endif // VEHICLE_SIM_H
