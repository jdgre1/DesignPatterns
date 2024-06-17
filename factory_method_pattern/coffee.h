#ifndef COFFEE_H
#define COFFEE_H

#include <iostream>
namespace patterns
{

enum class CoffeeSize
{
    Small,
    Medium,
    Large
};

enum class MilkType
{
    Fullcream,
    Skinny,
    Oat,
    Soy
};

enum class CoffeeStrength
{
    Single,
    DoubleShot,
    TripleShot
};

class Coffee
{
public:
    Coffee(CoffeeSize size, MilkType type, CoffeeStrength strength, bool chocpowder = false)
        : m_size(size)
        , m_milkType(type)
        , m_strength(strength)
        , m_chocpowder(chocpowder)
        , m_product("")
    {
    }
    virtual ~Coffee() = default;  // Virtual destructor to ensure proper cleanup of derived classes
    virtual std::string brew() const = 0;

    std::string CoffeeSizeToString() const
    {
        switch (m_size) {
        case CoffeeSize::Small: return ", small";
        case CoffeeSize::Medium: return ", medium";
        case CoffeeSize::Large: return ", large";
        default: {
            std::cout << "\nSize does not exist";
            return "";
        }
        }
    }

    std::string MilkTypeToString() const
    {
        switch (m_milkType) {
        case MilkType::Fullcream: return ", full-cream";
        case MilkType::Skinny: return ", skinny";
        case MilkType::Oat: return ", oat";
        case MilkType::Soy: return ", soy";
        default: {
            std::cout << "\nMilk type does not exist";
            return "";
        }
        }
    }

    std::string CoffeeStrengthToString() const
    {
        switch (m_strength) {
        case CoffeeStrength::Single: return ", single";
        case CoffeeStrength::DoubleShot: return ", double-shot";
        case CoffeeStrength::TripleShot: return ", triple-shot";
        default: {
            std::cout << "\nStrength option not available.";
            return "";
        }
        }
    }

protected:
    CoffeeSize m_size;
    MilkType m_milkType;
    CoffeeStrength m_strength;
    bool m_chocpowder;
    mutable std::string m_product;
};

}  // namespace patterns
#endif