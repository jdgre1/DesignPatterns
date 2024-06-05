#include <entity_manager.h>

namespace patterns
{

size_t FIELD_HEIGHT_MAX = 600;
size_t FIELD_WIDTH_MAX = 800;

// EntityManager* EntityManager::instancePtr = NULL;  

EntityManager& EntityManager::GetInstance()
{
    static EntityManager em;
    return em;
}

Entity EntityManager::CreateEntity()
{
    assert(m_entitiesAvailable.size() != 0);
    Entity id = m_entitiesAvailable.front();
    m_entitiesAvailable.pop_back();
    mEntityToIndexMap[id] = m_numEntities;
    mIndexToEntityMap[m_numEntities] = id;
    AddEntityToComponentRegister();
    return id;
}

void EntityManager::DeleteEntity(Entity e)
{
    assert(e < MAX_NUM_ENTITIES);
    m_entitiesAvailable.push_back(e);
    RemoveEntityFromComponentRegister(e);
}


void EntityManager::AddEntityToComponentRegister()
{
    m_componentRegister.entityXYComponents[m_numEntities].setPosition(0, 0);
    m_componentRegister.entityVelocityComponents[m_numEntities].setVelocity(0.0, 0.0);
    m_numEntities++;
}

void EntityManager::AddEntityToPlayerRegister(Entity newId,std::shared_ptr<Player> p)
{
    m_entityPlayerRegister[newId] = p;
}

void EntityManager::AddEntityToCharacterRegister(Entity newId, Character* c)
{
    m_entityCharacterRegister[newId] = c;
}

void EntityManager::RemoveEntityFromComponentRegister(Entity e)
{
    size_t entityToRemoveIndex = mEntityToIndexMap[e];
    m_componentRegister.entityXYComponents[entityToRemoveIndex] 
        = m_componentRegister.entityXYComponents[m_numEntities];
    m_componentRegister.entityVelocityComponents[entityToRemoveIndex] 
        = m_componentRegister.entityVelocityComponents[m_numEntities];

    mIndexToEntityMap[m_numEntities];

    // Update map
    // Replacing entity with that at end of array
    Entity entityAtEndOfArray = mIndexToEntityMap[m_numEntities];
    mIndexToEntityMap[entityToRemoveIndex] = entityAtEndOfArray;
    mEntityToIndexMap[entityAtEndOfArray] = entityToRemoveIndex;

    // Remove redundancies
    mEntityToIndexMap.erase(e);
    mIndexToEntityMap.erase(m_numEntities);

    m_numEntities--;
}

std::shared_ptr<Player> EntityManager::GetPlayerByEntityId(Entity e)
{
    return m_entityPlayerRegister[e];
}

} // namespace patterns
