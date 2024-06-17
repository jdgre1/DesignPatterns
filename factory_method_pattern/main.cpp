#include <coffee_machine.h>

int main()
{   
    // 1. Declare Coffee Factory for the Coffee Machine
    std::unique_ptr<patterns::CoffeeFactory> cf = std::make_unique<patterns::CoffeeFactory>();

    // 2. Coffee Machine
    patterns::CoffeeMachine& coffeeMachine = patterns::CoffeeMachine::GetInstance();
    coffeeMachine.SetCoffeeFactory(std::move(cf));

    // 3. Create an empty order of coffees
    std::vector<std::shared_ptr<patterns::Coffee>> coffees;
    
    // 4. Prepare coffees
    coffeeMachine.PrepareOrder(coffees);

    // 5. Display order
    coffeeMachine.DisplayOrder(coffees);

    return 0;
}