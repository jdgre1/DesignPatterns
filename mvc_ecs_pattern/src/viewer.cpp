#include <viewer.h>
namespace patterns
{

Viewer& Viewer::GetInstance(EntityManager& entityMngr, size_t fieldHeight, size_t fieldWidth)
{
    static Viewer viewer(entityMngr, fieldHeight, fieldWidth);
    return viewer;
}

Viewer::Viewer(EntityManager& entityMngr, size_t fieldHeightParm, size_t fieldWidthParam) 
    : m_em(entityMngr)
    , m_height(fieldHeightParm)
    , m_width(fieldWidthParam)
{}

bool Viewer::RenderFrame()
{   
    // Refresh frame
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);
    SDL_RenderClear(m_renderer);

    // Get position and rotation of player:
    EntityManager::ComponentRegister& compRegister = m_em.GetComponentRegister();

    // 1. Loop over each entity
    std::unordered_map<Entity, std::shared_ptr<Player>>& entityPlayerRegister = m_em.GetEntityPlayerRegister();
    size_t count = 0;
    for (auto& it : entityPlayerRegister) {
        Entity e = it.first;

        auto& entityXY = compRegister.entityXYComponents[e];
        Utils::drawCircle(m_renderer, entityXY.posX, entityXY.posY, 50, playerColor);

        std::shared_ptr<Player> p = m_em.GetPlayerByEntityId(e);
        double angle = p->GetAngleOfRotation();
        int lineEndX = entityXY.posX +  50.0 * cos(angle * M_PI / 180.0f);
        int lineEndY = entityXY.posY +  50.0 * sin(angle * M_PI / 180.0f);
        SDL_SetRenderDrawColor(m_renderer, 0, 0, 255, 255);
        SDL_RenderDrawLine(m_renderer, entityXY.posX, entityXY.posY, lineEndX, lineEndY);
        double xSquared = (entityXY.posX - lineEndX) * (entityXY.posX - lineEndX);
        double ySquared = (entityXY.posY - lineEndY) * (entityXY.posY - lineEndY);
        double lineLength = std::sqrt(xSquared + ySquared);
        // std::cout << "\n\nangle is " << angle;
        // std::cout << "\nlineLength is " << lineLength;
        // std::cout << "\nlineEndX is " << lineEndX;
        // std::cout << "\nlineEndY is " << lineEndY;
        // std::cout << "\nentityXY.posX is " << entityXY.posX;
        // std::cout << "\nentityXY.posY is " << entityXY.posY;

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