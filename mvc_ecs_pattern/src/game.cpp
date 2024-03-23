
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

void Game::initialiseMenu()
{
    m_menu->start();
    auto& characterVector = m_menu->getCharacters();
    for (std::shared_ptr<Character> pCharacter : characterVector) {
        Entity e = m_em->CreateEntity();

        if (std::dynamic_pointer_cast<Troll>(pCharacter)){
            std::cout << "\nTroll!";
        }
        else if (std::dynamic_pointer_cast<Orc>(pCharacter)){
            std::cout << "\nOrc!";
        }
        else{
            std::cout << "\nCharacter not recognised!!";
        }
    }
}

void Game::start()
{
    initialiseController();
    initialiseEntityManager();
    initialiseSystems();
    initialiseViewer();
    CreatePlayer();
    m_isRunning = true;
}

void Game::readInput()
{
    SDL_Event sdl_event;
    SDL_PollEvent(&sdl_event);

    if (sdl_event.type == SDL_KEYDOWN && !sdl_event.key.repeat) {
        std::cout << "sdl_event.type: " << sdl_event.type;

        const Uint8* keystates = SDL_GetKeyboardState(NULL);
        // Handle quit/escape, left, right, backward, and forward keys
        if (keystates[SDL_SCANCODE_ESCAPE] || sdl_event.type == SDL_QUIT) {
            m_isRunning = false;
        } else if (keystates[SDL_SCANCODE_LEFT]) {
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
        } else if (keystates[SDL_SCANCODE_W]) {
            m_controller->userInputIncreaseSpeed();
            // Handle backward key
        } else if (keystates[SDL_SCANCODE_S]) {
            m_controller->userInputDecreaseSpeed();
            // Handle backward key
        }
        // Process keystates and call controller functions
        // m_controller = Controller::GetInstance();
        // m_controller->userInputLeft();
    }
}

void Game::initialiseViewer()
{
    m_viewer = Viewer::GetInstance();
    m_viewer->SetWidth(m_width);
    m_viewer->SetHeight(m_height);
    m_viewer->Create();
}

void Game::initialiseSystems() { m_movementSystem = MovementSystem::GetInstance(); }

void Game::initialiseController() { m_controller = Controller::GetInstance(); }

Game::Game(std::size_t width, std::size_t height)
    : m_width(width)
    , m_height(height)
    , m_menu(std::make_unique<Menu>())
{
}

void Game::CreatePlayer()
{
    Entity e = m_em->CreateEntity();
    m_controller->SetPlayerId(e);
    Player* p = m_controller->GetPlayer();
    m_em->AddEntityToPlayerRegister(e, p);

    // m_em->
}

void Game::initialiseEntityManager() { m_em = EntityManager::GetInstance(); }

void Game::update() { m_movementSystem->update(); }

void Game::render() { m_viewer->RenderFrame(m_em); }

Game::~Game()
{
    // SDL_DestroyWindow(m_window);
    SDL_Quit();
}

} // namespace patterns