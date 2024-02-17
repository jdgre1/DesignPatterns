#include <entity_manager.h>

namespace patterns
{

size_t FIELD_HEIGHT_MAX = 60;
size_t FIELD_WIDTH_MAX = 100;

EntityManager* EntityManager::instancePtr = NULL;  

EntityManager* EntityManager::GetInstance()
{
    if (instancePtr == NULL) {
        instancePtr = new EntityManager();
    }
    return instancePtr;
}

Entity EntityManager::CreateEntity()
{
    assert(m_entitiesAvailable.size() != 0);
    Entity id = m_entitiesAvailable.front();
    m_entitiesAvailable.pop_back();
    AddEntityToComponentRegister(id);
    return id;
}

void EntityManager::DeleteEntity(Entity e)
{
    assert(e < MAX_NUM_ENTITIES);
    m_entitiesAvailable.push_back(e);
    RemoveEntityFromComponentRegister(e);
}


void EntityManager::AddEntityToComponentRegister(Entity newId)
{
    m_componentRegister.entityXYComponents[newId].setPosition(0, 0);
    m_componentRegister.entityVelocityComponents[newId].setVelocity(0.0, 0.0);
}

void EntityManager::AddEntityToPlayerRegister(Entity newId, Player* p)
{
    m_entityPlayerRegister[newId] = p;
}

void EntityManager::RemoveEntityFromComponentRegister(Entity e)
{
    m_componentRegister.entityXYComponents.erase(e);
    m_componentRegister.entityVelocityComponents.erase(e);
}

Player* EntityManager::GetPlayerByEntityId(Entity e)
{
    return m_entityPlayerRegister[e];
}

} // namespace patterns
