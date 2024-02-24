#include <viewer.h>
namespace patterns
{

Viewer* Viewer::instancePtr = NULL;

Viewer* Viewer::GetInstance()
{
    if (instancePtr == NULL) {
        instancePtr = new Viewer();
    }
    return instancePtr;
}

bool Viewer::RenderFrame(EntityManager* em)
{   
    // Refresh frame
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);
    SDL_RenderClear(m_renderer);

    // Get position and rotation of player:
    EntityManager::ComponentRegister& compRegister = em->GetComponentRegister();

    // 1. Loop over each entity
    std::unordered_map<Entity, Player*>& entityPlayerRegister = em->GetEntityPlayerRegister();
    size_t count = 0;
    for (auto& it : entityPlayerRegister) {
        Entity e = it.first;

        auto& entityXY = compRegister.entityXYComponents[e];
        Utils::drawCircle(m_renderer, entityXY.posY, entityXY.posX, 50, playerColor);

        Player* p = em->GetPlayerByEntityId(e);
        double angle = p->GetAngleOfRotation();
        int lineEndX = entityXY.posX +  50 * cos(angle * M_PI / 180);
        int lineEndY = entityXY.posY +  50 * sin(angle * M_PI / 180);
        SDL_SetRenderDrawColor(m_renderer, 0, 0, 255, 255);
        SDL_RenderDrawLine(m_renderer, entityXY.posY, entityXY.posX, lineEndX, lineEndY);
        // Utils::drawCircle(m_renderer, entityXY.posY, entityXY.posY, 50);
    }

    SDL_RenderPresent(m_renderer);
    SDL_Delay(10);

    // Draw player
    // void drawCircle(SDL_Renderer *renderer, int x, int y, int radius, SDL_Color color)
    // drawCircle(m_renderer, )

    return true;
}

void Viewer::Create()
{
    m_window =
        SDL_CreateWindow("sdl window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, m_width, m_height, NULL);

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

} // namespace patterns