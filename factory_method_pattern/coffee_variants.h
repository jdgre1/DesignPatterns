#ifndef COFFEE_VARIANTS_H
#define COFFEE_VARIANTS_H

#include <coffee.h>
namespace patterns
{
class Cappucino : public Coffee
{
public:
    Cappucino(CoffeeSize size, MilkType type, CoffeeStrength strength, bool chocpowder = true)
        : Coffee(size, type, strength, chocpowder)
    {
        // Additional initialization specific to Cappuccino, if any
        m_product ="Cappucinco";
    }

    std::string brew() const override
    {
        return m_product + CoffeeSizeToString() + CoffeeStrengthToString() + MilkTypeToString();
    }
};

class Latte : public Coffee
{
public:
    Latte(CoffeeSize size, MilkType type, CoffeeStrength strength, bool chocpowder = false)
        : Coffee(size, type, strength, chocpowder)

    {
       m_product = "Latte";
    }

    std::string brew() const override
    {
        return m_product + CoffeeSizeToString() + CoffeeStrengthToString() + MilkTypeToString();
    }
};

class Decaf : public Coffee
{
public:
    Decaf(CoffeeSize size, MilkType type, CoffeeStrength strength, bool chocpowder = false)
        : Coffee(size, type, strength, chocpowder)
    {
        // Additional initialization specific to Cappuccino, if any
        m_product ="Decaf";
    }

    std::string brew() const override
    {
        return m_product + CoffeeSizeToString() + MilkTypeToString();
    }
};
}  // namespace patterns
#endif