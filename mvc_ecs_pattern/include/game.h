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
#include <memory>
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

        // static Game* GetInstance(std::size_t width, std::size_t height);
        static Game& GetInstance(EntityManager& entityMngr, Menu& menu, Controller& cntlr, Viewer& viewer, MovementSystem& moveSys, std::shared_ptr<Player> plyr);

        void start();
        void readInput();
        bool isRunning(){ return m_isRunning;}
        void update();
        void render();
        void setIsRunning(bool isRunning){m_isRunning = isRunning;}

    private:
        void initialiseMenu();
        void initialiseSystems();
        void initialiseViewer();
        void CreatePlayer();
        void loadCharacters();

        Game(EntityManager& entityMngr, Menu& menu, Controller& cntlr, Viewer& viewer, MovementSystem& moveSys, std::shared_ptr<Player> plyr);
        bool m_isRunning = false;
        std::size_t m_width;
        std::size_t m_height;
        static Game* instancePtr;

        // Should I make all of these unique_ptrs?
        EntityManager& m_em;
        Menu& m_menu;
        Controller& m_controller;
        SDL_Renderer* m_renderer;
        Viewer& m_viewer;
        MovementSystem& m_movementSystem;
        std::shared_ptr<Player> m_player;

        SDL_Window* m_window; 
        bool m_is_running;
};
} // namespace patterns
#endif  // GAME_H