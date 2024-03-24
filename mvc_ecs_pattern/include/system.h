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
        void SetRenderer(SDL_Renderer* renderer){
            m_renderer = renderer;
        }
        SDL_Point getNewStartingPoint(int condition, double x, double y, double angleRadians);


    private:
        MovementSystem(){}
        static MovementSystem* instancePtr;
        double m_angleIncrement = 20.0;
        SDL_Renderer* m_renderer;

};

} // namespace patterns
#endif  // SYSTEM_H
