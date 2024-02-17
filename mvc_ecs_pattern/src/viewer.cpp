#include <viewer.h>

namespace patterns {

Viewer* Viewer::instancePtr = NULL;  

Viewer* Viewer::GetInstance()
{
    if (instancePtr == NULL) {
        instancePtr = new Viewer();
    }
    return instancePtr;
}

bool Viewer::RenderFrame()
{
    return true;
}

void Viewer::Create()
{
    m_window = SDL_CreateWindow("sdl window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, m_width, m_height, NULL);

    if (m_window == NULL) {
        std::cout << "Could not create window: " << SDL_GetError() << '\n';
        // m_isRunning = false;
    }
    m_renderer = SDL_CreateRenderer(m_window, -1, 0);
    if (!m_renderer) {
        std::cout << "Error creating SDL renderer.\n";
        // m_isRunning = false;
    }

}

}