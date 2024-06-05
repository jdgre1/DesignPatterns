#pragma once

#ifndef MENU_H
#define MENU_H
#include <character_factory.h>
#include <entity_manager.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>

namespace patterns {

class Menu
{
    public:

        // // Delete copy constructor
        Menu(const Menu& obj) = delete; 
        /** * Singletons should not be assignable. */
        void operator=(const Menu&) = delete;

        static Menu& GetInstance();
        void start();
        std::vector<std::shared_ptr<Character>> const& getCharacters() const{   
            return m_characters;
        }

    private:
       
        Menu();

        SDL_Color playerColor = {120, 255, 120};
        size_t m_height;
        size_t m_width;
        SDL_Window* m_window; 
        SDL_Renderer* m_renderer;
        bool m_is_running;
        // std::vector<Orc> m_orcs;
        // std::vector<Troll> m_trolls;
        std::vector<std::shared_ptr<Character>> m_characters;
        std::unique_ptr<CharacterFactory> m_pCf;
        // std::unique_ptr<OrcFactory> m_of;


};
} // namespace patterns
#endif  // MENU_H
