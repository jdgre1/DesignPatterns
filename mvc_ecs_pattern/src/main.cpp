#include <components.h>
#include <game.h>
#include <entity_manager.h>
#include <system.h>


int main(int argc, char* argv[])
{
   // create a game on a 800x600 window
    patterns::Game* game = patterns::Game::GetInstance(800, 600);

    while(game->isRunning()) 
    {
        game->readInput();
        game->update();
        game->render();
    }
    return 0;
}


//TODO
// Fix compilation issues with 'Player not declared in this scope'
// Make const functions const variables / return const where possible