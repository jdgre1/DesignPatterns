#pragma once
#ifndef BUG_FACTORY_H
#define BUG_FACTORY_H

#include <memory>
#include <vector>

#include <bug.h>
namespace patterns
{


    class BugFactoryBase
    {

    public:
        virtual std::shared_ptr<Bug> CreateBug(BugType bugType, uint8_t bugSize, uint8_t speed, uint8_t strength) = 0;
    };

    class BugFactory : BugFactoryBase
    {
    public:
        std::shared_ptr<Bug> CreateBug(BugType bugType, uint8_t bugSize, uint8_t speed, uint8_t strength) override
        {
            switch (bugType)
            {
            case BugType::Alien:
                return std::shared_ptr<Bug>(
                    std::make_shared<AlienBug>(bugSize, speed, strength));

            case BugType::Zipper:
                return std::shared_ptr<Bug>(
                    std::make_shared<ZipperBug>(bugSize, speed, strength));

            case BugType::BigBertha:
                return std::shared_ptr<Bug>(
                    std::make_shared<BigBerthaBug>(bugSize, speed, strength));

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