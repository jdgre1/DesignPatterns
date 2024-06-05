#pragma once

#ifndef ORC_H
#define ORC_H

#include <character.h>

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <utils.h>


namespace patterns {

class Orc : public Character
{
    public:

        // Delete copy constructor
        // Orc(const Orc& obj) = delete; 
        // /** * Singletons should not be assignable. */
        // void operator=(const Orc&) = delete;
        // static Orc* GetInstance();
        Orc() {}

        ~Orc(){}
        void SetEntityId(Entity e){
            m_id = e;
        }
        Entity GetId(){
            return m_id;
        }

    private:
        Entity m_id;
};
} // namespace patterns
#endif  // ORC_H
