#pragma once

#ifndef SYSTEM_H
#define SYSTEM_H

namespace patterns{

class ISystem
{
    virtual void update() = 0;
};

class MovementSystem : ISystem
{   
    public:
        MovementSystem(const MovementSystem& obj) = delete; 
        /** * Singletons should not be assignable. */
        void operator=(const MovementSystem&) = delete;

        static MovementSystem* GetInstance();
        void update();

    private:
        MovementSystem(){}
        static MovementSystem* instancePtr;


};

} // namespace patterns
#endif  // SYSTEM_H
