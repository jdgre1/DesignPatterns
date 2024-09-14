#pragma once
#ifndef BUG_FACTORY_H
#define BUG_FACTORY_H

#include <memory>
#include <vector>

#include <bug.h>
namespace patterns
{
    enum class BugType
    {
        Alien,
        Zipper,
        BigBertha
    };

    class BugFactoryBase
    {

    public:
        virtual std::unique_ptr<Bug> CreateBug(BugType bugType, uint8_t bugSize, uint8_t speed, uint8_t strength) = 0;
    };

    class BugFactory : BugFactoryBase
    {
    public:
        std::unique_ptr<Bug> CreateBug(BugType bugType, uint8_t bugSize, uint8_t speed, uint8_t strength) override
        {
            switch (bugType)
            {
            case BugType::Alien:
                return std::unique_ptr<Bug>(
                    std::make_unique<AlienBug>(bugSize, speed, strength));

            case BugType::Zipper:
                return std::unique_ptr<Bug>(
                    std::make_unique<ZipperBug>(bugSize, speed, strength));

            case BugType::BigBertha:
                return std::unique_ptr<Bug>(
                    std::make_unique<BigBerthaBug>(bugSize, speed, strength));

            default:
            {
                std::cout << "\nCoffee type does not exist in factory";
                return nullptr;
            }
            }
        }
    };
} // namespace patterns
#endif