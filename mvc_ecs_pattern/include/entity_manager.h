#pragma once

#ifndef ENTITYMANAGER_H
#define ENTITYMANAGER_H

#include <utils.h>

#include <character.h>
#include <components.h>
#include <player.h>

#include <memory>
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
            std::array<Components::VelocityComponent, MAX_NUM_ENTITIES> entityVelocityComponents;
            std::array<Components::XYComponent, MAX_NUM_ENTITIES> entityXYComponents;
        };

        // Delete copy constructor
        EntityManager(const EntityManager& obj) = delete; 
        /** * Singletons should not be assignable. */
        void operator=(const EntityManager&) = delete;

        static EntityManager& GetInstance();

        Entity CreateEntity();
        void DeleteEntity(Entity e);
        void AddEntityToComponentRegister();
        void AddEntityToPlayerRegister(Entity newId, std::shared_ptr<Player> pl);
        void AddEntityToCharacterRegister(Entity newId, Character* p);
        void RemoveEntityFromComponentRegister(Entity e);
        ComponentRegister& GetComponentRegister()
        {
            return m_componentRegister;
        }
        
        std::unordered_map<Entity, std::shared_ptr<Player>>& GetEntityPlayerRegister(){ return m_entityPlayerRegister;}
        std::shared_ptr<Player> GetPlayerByEntityId(Entity e);

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
        std::unordered_map<Entity, std::shared_ptr<Player>> m_entityPlayerRegister;
        std::unordered_map<Entity, Character*> m_entityCharacterRegister;

        // See https://austinmorlan.com/posts/entity_component_system/

        // Map from an entity ID to an array index.
        std::unordered_map<Entity, size_t> mEntityToIndexMap;

        // Map from an array index to an entity ID.
        std::unordered_map<size_t, Entity> mIndexToEntityMap;

};
} // namespace patterns

#endif  // ENTITYMANAGER_H