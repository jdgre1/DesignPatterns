#pragma once

#ifndef SYSTEM_H
#define SYSTEM_H

namespace patterns{

enum class NewStartingPosition {
    TOP,
    BOTTOM,
    RIGHT_SIDE,
    LEFT_SIDE
};

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

        static MovementSystem& GetInstance(EntityManager& entityMngr);
        void update();
        void SetRenderer(SDL_Renderer* renderer){
            m_renderer = renderer;
        }
        SDL_Point getNewStartingPoint(NewStartingPosition pos, double x, double y, double angleRadians);


    private:
        MovementSystem(EntityManager& entityMngr);

        double m_angleIncrement = 20.0;
        EntityManager& m_em;
        SDL_Renderer* m_renderer;

};

} // namespace patterns
#endif  // SYSTEM_H
