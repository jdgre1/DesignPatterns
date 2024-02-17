#include <system.h>
#include <entity_manager.h>

namespace patterns{

MovementSystem* MovementSystem::instancePtr = NULL;

MovementSystem* MovementSystem::GetInstance()
{
    if (instancePtr == NULL) {
        instancePtr = new MovementSystem();
    }
    return instancePtr;
}
void MovementSystem::update()
{
    EntityManager* entityManager = EntityManager::GetInstance();
    EntityManager::ComponentRegister& compRegister = entityManager->GetComponentRegister();

    // Player* player = entityManager->GetPlayerByEntityId(e);
    
    // 1. Loop over each entity
    std::unordered_map<Entity, Player*>& entityPlayerRegister = entityManager->GetEntityPlayerRegister();

    for (auto& it: entityPlayerRegister) {
        Entity e = it.first;
        Player* p = it.second;

        KeyboardCommand k = p->GetLastKeyboardCommand();

        switch(k){
            case KeyboardCommand::LEFT:
                std::cout << "left";
                // alter angle and calculate new position of XY
                // alter angle and calculate new velocity (direction)

                break;
            default:

                std::cout << "default";
        }

        auto& entityXY = compRegister.entityXYComponents[e];
        auto& entityVel = compRegister.entityVelocityComponents[e];
    }



}


}