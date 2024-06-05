#include <components.h>
#include <entity_manager.h>
#include <game.h>
#include <player.h>
#include <system.h>
#include <viewer.h>

int main(int argc, char* argv[])
{
    auto pl = std::make_shared<patterns::Player>();
    patterns::EntityManager& em = patterns::EntityManager::GetInstance();
    patterns::Viewer& viewer = patterns::Viewer::GetInstance(em, patterns::FIELD_HEIGHT_MAX, patterns::FIELD_WIDTH_MAX);
    patterns::Controller& cntrlr = patterns::Controller::GetInstance(pl);
    patterns::MovementSystem& ms = patterns::MovementSystem::GetInstance(em);
    patterns::Menu& menu = patterns::Menu::GetInstance();
    patterns::Game& game = patterns::Game::GetInstance(em, menu,  cntrlr, viewer, ms, pl);

    game.start();
    while (game.isRunning()) {
        game.readInput();
        game.update();
        game.render();
    }
    return 0;
}

// TODO
//  Make const functions const variables / return const where possible