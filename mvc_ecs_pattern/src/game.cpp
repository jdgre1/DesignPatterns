
#include <game.h>
#include <player.h>

namespace patterns
{

Game& Game::GetInstance(EntityManager& entityMngr, Menu& menu, Controller& cntlr, Viewer& viewer, MovementSystem& moveSys, std::shared_ptr<Player> plyr)
{
    static Game game(entityMngr, menu, cntlr, viewer, moveSys, plyr);
    return game;
}

void Game::initialiseMenu()
{
    m_menu.start();
}

void Game::start()
{
    initialiseViewer();
    // initialiseMenu();
    initialiseSystems();
    CreatePlayer();
    // loadCharacters();
    m_isRunning = true;
}

void Game::loadCharacters()
{
    auto& characterVector = m_menu.getCharacters();
    for (std::shared_ptr<Character> pCharacter : characterVector) {
        Entity e = m_em.CreateEntity();

        if (std::dynamic_pointer_cast<Troll>(pCharacter)){
            std::cout << "\nTroll!";
            // pCharacter->initialisePosition();
        }
        else if (std::dynamic_pointer_cast<Orc>(pCharacter)){
            std::cout << "\nOrc!";
            // pCharacter->initialisePosition();
        }
        else{
            std::cout << "\nCharacter not recognised!!";
        }
        m_em.AddEntityToComponentRegister();
    }
}

void Game::readInput()
{
    SDL_Event sdl_event;
    SDL_PollEvent(&sdl_event);

    if (sdl_event.type == SDL_KEYDOWN && !sdl_event.key.repeat) {
        const Uint8* keystates = SDL_GetKeyboardState(NULL);
        // Handle quit/escape, left, right, backward, and forward keys
        if (keystates[SDL_SCANCODE_ESCAPE] || sdl_event.type == SDL_QUIT) {
            m_isRunning = false;
        } else if (keystates[SDL_SCANCODE_LEFT]) {
            m_controller.userInputLeft();
            // Handle left key
        } else if (keystates[SDL_SCANCODE_RIGHT]) {
            m_controller.userInputRight();
            // Handle right key
        } else if (keystates[SDL_SCANCODE_UP]) {
            m_controller.userInputForwards();
            // Handle forward key
        } else if (keystates[SDL_SCANCODE_DOWN]) {
            m_controller.userInputBackwards();
            // Handle backward key
        } else if (keystates[SDL_SCANCODE_W]) {
            m_controller.userInputIncreaseSpeed();
            // Handle backward key
        } else if (keystates[SDL_SCANCODE_S]) {
            m_controller.userInputDecreaseSpeed();
            // Handle backward key
        }
        // Process keystates and call controller functions
        // m_controller = Controller::GetInstance();
        // m_controller->userInputLeft();
    }
}

void Game::initialiseViewer()
{
    m_viewer.Create();
    
}

void Game::initialiseSystems() { m_movementSystem.SetRenderer(m_viewer.GetRenderer()); }

Game::Game(EntityManager& entityMngr, Menu& menu, Controller& cntlr, Viewer& viewer, MovementSystem& moveSys, std::shared_ptr<Player> plyr)
    : m_em(entityMngr)
    , m_menu(menu)
    , m_controller(cntlr)
    , m_viewer(viewer)
    , m_movementSystem(moveSys)
    , m_player(plyr)
{
}

void Game::CreatePlayer()
{
    Entity e = m_em.CreateEntity();
    m_controller.SetPlayerId(e);
    m_em.AddEntityToPlayerRegister(e, m_player);
}


void Game::update() { m_movementSystem.update(); }

void Game::render() { m_viewer.RenderFrame(); }

Game::~Game()
{
    // SDL_DestroyWindow(m_window);
    SDL_Quit();
}

} // namespace patterns