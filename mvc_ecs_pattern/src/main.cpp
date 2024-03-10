#include <components.h>
#include <entity_manager.h>
#include <game.h>
#include <menu.h>
#include <system.h>

int main(int argc, char* argv[])
{
   
    patterns::Game* game = patterns::Game::GetInstance(patterns::FIELD_HEIGHT_MAX, patterns::FIELD_WIDTH_MAX);

    while (game->isRunning()) {
        game->readInput();
        game->update();
        game->render();
    }
    return 0;
}

// TODO
//  Fix compilation issues with 'Player not declared in this scope'
//  Make const functions const variables / return const where possible