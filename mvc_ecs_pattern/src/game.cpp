
#include <game.h>
#include <player.h>

namespace patterns
{
Game* Game::instancePtr = NULL;

Game* Game::GetInstance(std::size_t width, std::size_t height)
{
    if (instancePtr == NULL) {
        instancePtr = new Game(width, height);
    }
    return instancePtr;
}

void Game::readInput()
{
    SDL_Event sdl_event;
    SDL_PollEvent(&sdl_event);
    const Uint8* keystates = SDL_GetKeyboardState(NULL);

    // Handle quit/escape, left, right, backward, and forward keys
    if (keystates[SDL_SCANCODE_ESCAPE] || sdl_event.type == SDL_QUIT) {
        m_isRunning = false;
    }
    else if (keystates[SDL_SCANCODE_LEFT]) {
        m_controller->userInputLeft();
        // Handle left key
    } else if (keystates[SDL_SCANCODE_RIGHT]) {
        m_controller->userInputRight();
        // Handle right key
    } else if (keystates[SDL_SCANCODE_UP]) {
        m_controller->userInputForwards();
        // Handle forward key
    } else if (keystates[SDL_SCANCODE_DOWN]) {
        m_controller->userInputBackwards();
        // Handle backward key
    } else if (keystates[SDL_SCANCODE_W]){
        m_controller->userInputIncreaseSpeed();
        // Handle backward key
    } else if (keystates[SDL_SCANCODE_S]){
        m_controller->userInputDecreaseSpeed();
        // Handle backward key
    }
    // Process keystates and call controller functions
    m_controller = Controller::GetInstance();
    // m_controller->userInputLeft();
}

void Game::initialiseViewer()
{
    m_viewer = Viewer::GetInstance();
    m_viewer->SetWidth(m_width);
    m_viewer->SetHeight(m_height);
    m_viewer->Create();
}

void Game::initialiseSystems()
{   
    m_movementSystem = MovementSystem::GetInstance();
}

void Game::initialiseController()
{   
    m_controller = Controller::GetInstance();
}

Game::Game(std::size_t width, std::size_t height)
    : m_width(width)
    , m_height(height)
{
    initialiseController();
    initialiseEntityManager();
    initialiseSystems();
    initialiseViewer();
    CreatePlayer();
    m_isRunning = true;
}

void Game::CreatePlayer()
{
    Entity e = m_em->CreateEntity();
    m_em->AddEntityToComponentRegister(e);
    m_controller->SetPlayerId(e);
    Player* p = m_controller->GetPlayer();
    m_em->AddEntityToPlayerRegister(e, p);

    // m_em->

}

void Game::initialiseEntityManager()
{
    m_em = EntityManager::GetInstance();

}

void Game::update(){ 

    m_movementSystem->update();
}

void Game::render() {}

Game::~Game()
{
    // SDL_DestroyWindow(m_window);
    SDL_Quit();
}

} // namespace patterns