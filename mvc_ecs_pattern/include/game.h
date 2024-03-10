#pragma once

#ifndef GAME_H
#define GAME_H

// local
#include <controller.h>
#include <entity_manager.h>
#include <menu.h>
#include <system.h>
#include <viewer.h>

// third-party
#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>



namespace patterns {

class Game
{
    public:

        // Delete copy constructor
        Game(const Game& obj) = delete; 
        /** * Singletons should not be assignable. */
        void operator=(const Game&) = delete;
        ~Game();

        static Game* GetInstance(std::size_t width, std::size_t height);

        void start();
        void readInput();
        bool isRunning(){ return m_isRunning;}
        void update();
        void render();
        void setIsRunning(bool isRunning){m_isRunning = isRunning;}

    private:
        void initialiseViewer();
        void initialiseController();
        void initialiseEntityManager();
        void initialiseSystems();
        void CreatePlayer();

        explicit Game(std::size_t width, std::size_t height);
        bool m_isRunning = false;
        std::size_t m_width;
        std::size_t m_height;
        static Game* instancePtr;

        std::unique_ptr<Menu> m_menu;
        Controller* m_controll1er;
        MovementSystem* m_movementSystem;
        EntityManager* m_em;
        SDL_Renderer* m_renderer;
        Viewer* m_viewer;
        SDL_Window* m_window; 
        bool m_is_running;
};
} // namespace patterns
#endif  // GAME_H