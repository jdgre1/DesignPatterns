#ifndef BUG_H
#define BUG_H

#include <iostream>

namespace patterns 
{
class Bug
{
public:
    Bug(uint8_t size, uint8_t speed, uint8_t strength)
        : m_size(size)
        , m_speed(speed)
        , m_strength(strength)
    {
    }

    virtual std::string attack() const = 0;

    virtual ~Bug() = default;  // Virtual destructor to ensure proper cleanup of derived classes

protected:
    uint8_t m_size;
    uint8_t m_speed;
    uint8_t m_strength;
    std::string m_type;
};


class AlienBug : Bug
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

class ZipperBug : Bug
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

class BigBerthaBug : Bug
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