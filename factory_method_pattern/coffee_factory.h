#ifndef COFFEE_FACTORY_H
#define COFFEE_FACTORY_H

#include <coffee.h>
#include <coffee_variants.h>
#include <memory>
#include <vector>

namespace patterns
{
enum class CoffeeType
{
    CappucinoType,
    DecafType,
    LatteType,
};

class CoffeeFactoryBase
{

public:
    virtual std::unique_ptr<Coffee> MakeCoffee(CoffeeType coffeeType, CoffeeSize size,
            MilkType type, CoffeeStrength strength, bool chocpowder = false) = 0;
};

class CoffeeFactory : CoffeeFactoryBase
{
public:
    std::unique_ptr<Coffee> MakeCoffee(CoffeeType coffeeType, CoffeeSize size, MilkType type,
            CoffeeStrength strength, bool chocpowder = false) override
    {
        switch (coffeeType) {
        case CoffeeType::CappucinoType:
            return std::unique_ptr<Coffee>(
                    std::make_unique<Cappucino>(size, type, strength, chocpowder));

        case CoffeeType::DecafType:
            return std::unique_ptr<Coffee>(
                    std::make_unique<Decaf>(size, type, strength, chocpowder));

        case CoffeeType::LatteType:
            return std::unique_ptr<Coffee>(
                    std::make_unique<Latte>(size, type, strength, chocpowder));

        default: {
            std::cout << "\nCoffee type does not exist in factory";
            return nullptr;
        }
        }
    }
};
}  // namespace patterns
#endif