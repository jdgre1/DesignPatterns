#ifndef COFFEE_MACHINE_H
#define COFFEE_MACHINE_H

#include <coffee.h>
#include <coffee_factory.h>
#include <coffee_variants.h>
#include <memory>
#include <vector>

namespace patterns
{

class CoffeeMachine
{

public:
    // Delete copy constructor
    CoffeeMachine(const CoffeeMachine& obj) = delete;
    /** * Singletons should not be assignable. */
    void operator=(const CoffeeMachine&) = delete;
    ~CoffeeMachine(){};

    static CoffeeMachine& GetInstance()
    {
        static CoffeeMachine coffeeMachine;
        return coffeeMachine;
    }

    void SetCoffeeFactory(std::unique_ptr<CoffeeFactory> cf)
    {
        m_cf = std::move(cf);
    }

    void prepareOrder(std::vector<std::shared_ptr<Coffee>>& order)
    {
        bool stillOrdering = true;
        patterns::CoffeeType coffeeType;
        patterns::CoffeeStrength strength;
        patterns::CoffeeSize size;
        patterns::MilkType milkType;

        std::cout << "\nPlease select your coffee according to the following:";

        while (stillOrdering) {
            bool invalidInput = true;
            int val = -1;
            while (invalidInput) {
                std::cout << "Please select your coffee (Cappucino = 0, Decaf = 1, Latte = "
                             "2)\n";

                std::cin >> val;
                if (val > 2 || val < 0)
                    std::cout << "Invalid input: Please try again\n";
                else {
                    coffeeType = static_cast<CoffeeType>(val);
                    invalidInput = false;
                }
            }

            invalidInput = true;
            while (invalidInput) {
                std::cout << "Please select your size (Small = 0, Medium = 1, Large = 2)\n";
                std::cin >> val;
                if (val > 2 || val < 0)
                    std::cout << "Invalid input: Please try again\n";
                else {
                    size = static_cast<CoffeeSize>(val);
                    invalidInput = false;
                }
            }

            invalidInput = true;
            while (invalidInput) {
                std::cout << "Please select your preferred-milk (Fullcream = 0, Skinny = 1, Oat = "
                             "2, Soy = 3)\n";
                std::cin >> val;
                if (val > 3 || val < 0)
                    std::cout << "Invalid input: Please try again\n";
                else {
                    milkType = static_cast<MilkType>(val);
                    invalidInput = false;
                }
            }

            if (coffeeType != CoffeeType::DecafType) {
                invalidInput = true;
                while (invalidInput) {
                    std::cout << "Please select your coffee strength (1 shot = 1, 2 shots = 2, 3 "
                                 "shots = 3)\n";
                    std::cin >> val;
                    if (val > 3 || val < 1)
                        std::cout << "Invalid input: Please try again\n";
                    else {
                        invalidInput = false;
                        val-=1;
                        strength = static_cast<CoffeeStrength>(val);
                    }
                }
            }

            std::shared_ptr<Coffee> coffeeOrdered =
                    m_cf->MakeCoffee(coffeeType, size, milkType, strength);
            order.push_back(std::move(coffeeOrdered));
            invalidInput = true;
            while (invalidInput) {
                std::cout << "Are you finished ordering? 1 - yes, 0 - no\n";
                std::cin >> val;
                if (val > 1 || val < 0)
                    std::cout << "Invalid input: Please try again\n";
                else {
                    invalidInput = false;
                    stillOrdering = !(static_cast<bool>(val));
                }
            }
        }
    };

    void displayOrder(std::vector<std::shared_ptr<Coffee>>& coffeeOrder)
    {
        std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\nPreparing orders:\n\n";
        size_t count = 1;
        for (auto c : coffeeOrder) {
            if (auto cappucino = std::dynamic_pointer_cast<Cappucino>(c)) {
                std::cout << count++ << ". " << cappucino->brew() << std::endl;
            } else if (auto latte = std::dynamic_pointer_cast<Latte>(c)) {
                std::cout << count++ << ". " << latte->brew() << std::endl;
            } else if (auto decaf = std::dynamic_pointer_cast<Decaf>(c)) {
                std::cout << count++ << ". " << decaf->brew() << std::endl;
            }
        }
    }

private:
    std::unique_ptr<patterns::CoffeeFactory> m_cf;
    CoffeeMachine()
    {
    }
};

}  // namespace patterns

#endif
