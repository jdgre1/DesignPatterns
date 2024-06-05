#include <PlayerFactory.h>

namespace patterns
{
PlayerFactory* PlayerFactory::instancePtr = NULL;  


PlayerFactory* PlayerFactory::GetInstance()
{
    if (instancePtr == NULL) {
        instancePtr = new PlayerFactory();
    }
    return instancePtr;
}

PlayerFactory::PlayerFactory() {}




} // namespace patterns
