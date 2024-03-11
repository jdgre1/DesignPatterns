#pragma once

#ifndef ENTITYMANAGER_H
#define ENTITYMANAGER_H

#include <utils.h>

#include <components.h>
#include <player.h>

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <unordered_map>


namespace patterns {


class EntityManager
{
    public:

        struct ComponentRegister // ToDo: Needs to be array of indexes
        {
            // std::unordered_map<Entity, Components::XYComponent> entityXYComponents;
            // std::unordered_map<Entity, Components::VelocityComponent> entityVelocityComponents;

        };

        // Delete copy constructor
        EntityManager(const EntityManager& obj) = delete; 
        /** * Singletons should not be assignable. */
        void operator=(const EntityManager&) = delete;

        static EntityManager* GetInstance();

        Entity CreateEntity();
        void DeleteEntity(Entity e);
        void AddEntityToComponentRegister(Entity newId);
        void AddEntityToPlayerRegister(Entity newId, Player* p);
        void RemoveEntityFromComponentRegister(Entity e);
        ComponentRegister& GetComponentRegister()
        {
            return m_componentRegister;
        }
        
        std::unordered_map<Entity, Player*>& GetEntityPlayerRegister(){ return m_entityPlayerRegister;}
        Player* GetPlayerByEntityId(Entity e);

    private:

        EntityManager()
        {
            m_entitiesAvailable.resize(MAX_NUM_ENTITIES);
            fill(m_entitiesAvailable.begin(), m_entitiesAvailable.end(), m_numEntities++);
            m_numEntities = 0;
        }

        uint32_t m_numEntities = 0;
        std::vector<Entity> m_entitiesAvailable;
        ComponentRegister m_componentRegister;
        static EntityManager* instancePtr;
        std::unordered_map<Entity, Player*> m_entityPlayerRegister;

        // TODO - see https://austinmorlan.com/posts/entity_component_system/
        // Should be an array not a map of velocity components to entity indices
        // Then a map to map entities to indices
        std::array<T, MAX_ENTITIES> mComponentArray;

        // Map from an entity ID to an array index.
        std::unordered_map<Entity, size_t> mEntityToIndexMap;

        // Map from an array index to an entity ID.
        std::unordered_map<size_t, Entity> mIndexToEntityMap;

};
} // namespace patterns

#endif  // ENTITYMANAGER_H