#pragma once

#ifndef VIEWER_H
#define VIEWER_H

#include <entity_manager.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <cassert>
#include <stdio.h>
#include <stdint.h>
#include <iostream>

namespace patterns {

class Viewer
{
    public:

        // Delete copy constructor
        Viewer(const Viewer& obj) = delete; 
        /** * Singletons should not be assignable. */
        void operator=(const Viewer&) = delete;

        static Viewer& GetInstance(EntityManager& entityMngr, size_t height, size_t width);
        void Create();
        void SetWidth(size_t width){m_width = width;}
        void SetHeight(size_t height){m_height = height;};
        bool RenderFrame();
        SDL_Renderer* GetRenderer(){
            return m_renderer;
        }
        

    private:
        Viewer(EntityManager& entityMngr, size_t height, size_t width);

        SDL_Color playerColor = {120, 255, 120};
        EntityManager& m_em;
        size_t m_height;
        size_t m_width;
        SDL_Window* m_window; 
        SDL_Renderer* m_renderer;
        bool m_is_running;

};
} // namespace patterns
#endif  // VIEWER_H
