#pragma once
#ifndef BUG_H
#define BUG_H

#include <iostream>
#include <components.h>

namespace patterns 
{

enum class BugType
{
    Alien,
    Zipper,
    BigBertha
};

class Bug
{
public:
    Bug(uint8_t size, uint8_t speed, uint8_t strength)
        : m_size(size)
        , m_speed(speed)
        , m_strength(strength)
    {
    }

    uint8_t getSize(){ return m_size; }

    Components::XYComponent getPos(){ return m_xyComponent; }

    uint8_t getSpeed(){ return m_speed; }

    uint8_t getStrength(){ return m_strength; }

    std::string getType(){ return m_type; }


    virtual std::string attack() const = 0;

    virtual ~Bug() = default;  // Virtual destructor to ensure proper cleanup of derived classes

protected:
    uint8_t m_size;
    uint8_t m_strength;
    uint8_t m_speed;
    Components::XYComponent m_xyComponent;
    Components::VelocityComponent m_velComponent;
    std::string m_type;
};


class AlienBug : public Bug
{
    public:
        AlienBug(uint8_t size, uint8_t speed, uint8_t strength)
        : Bug(size, speed, strength)
        {
            m_type = "Alien";
        }

        std::string attack() const override
        {
            return "Alien-attack!";
        }

};

class ZipperBug : public Bug
{
    public:
        ZipperBug(uint8_t size, uint8_t speed, uint8_t strength)
        : Bug(size, speed, strength)
        {
            m_type = "Zipper";
        }

        std::string attack() const override
        {
            return "Zipper-attack!";
        }

};

class BigBerthaBug : public Bug
{
    public:
        BigBerthaBug(uint8_t size, uint8_t speed, uint8_t strength)
        : Bug(size, speed, strength)
        {
            m_type = "BigBertha";
        }

        std::string attack() const override
        {
            return "BigBertha-attack!";
        }

};
}
#endif