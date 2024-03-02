#pragma once

#ifndef TROLL_H
#define TROLL_H

#include <character.h>

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <utils.h>


namespace patterns {

class Troll : public Character
{
    public:

        // Delete copy constructor
        // Troll(const Troll& obj) = delete; 
        // /** * Singletons should not be assignable. */
        // void operator=(const Troll&) = delete;
        // static Troll* GetInstance();
        Troll() {}

        ~Troll(){}
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
#endif  // TROLL_H
