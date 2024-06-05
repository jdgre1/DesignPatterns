#include <menu.h>
namespace patterns
{

Menu& Menu::GetInstance()
{
    static Menu menu;
    return menu;
}


Menu::Menu()
    : m_pCf(std::make_unique<CharacterFactory>())  //= std::make_unique<OrcFactory>()
    // : m_of(std::unique_ptr<OrcFactory>(new OrcFactory))  //= std::make_unique<OrcFactory>()
    // , m_tf(std::make_unique<TrollFactory>())  //= std::make_unique<OrcFactory>()
// , m_tf = std::make_unique<TrollFactory>()
{
    // MyObject() : ptr(std::unique_ptr<int>(new int)) 
}

// Menu::~Menu()
// {
    
// }

SDL_Color returnColourFromCode(size_t selectionId)
{
    switch (selectionId) {
    case 1: {
        SDL_Color blue = {255, 0, 0};
        return blue;
    }
    case 2: {
        SDL_Color green = {0, 255, 0};
        return green;
    }
    case 3: {
        SDL_Color red = {0, 0, 255};
        return red;
    }
    default: {
        SDL_Color black = {0, 0, 0};
        return black;
    }
    }
}

// Use cin for now:
void Menu::start()
{
    // ToDo: Error Checking
    // Menu menu GetInstance();
    size_t numOrcs = 0;
    std::cout << "\nPlease specify how many orcs will be on the screen: ";
    std::cin >> numOrcs;
    for (size_t i = 0; i < numOrcs; i++) {
        size_t colourCode = 0;
        std::cout << "\nPlease specify the colour of orc " << i + 1
                  << ": Either '1' for blue, '2' for green or '3' for red. ";
        std::cin >> colourCode;
        std::shared_ptr<Character> pOrc = m_pCf->MakeCharacter(CharacterTypes::OrcType);
        
        SDL_Color colour = returnColourFromCode(colourCode);
        pOrc->setColour(colour);
        m_characters.push_back(std::move(pOrc));
    }

    size_t numTrolls = 0;
    std::cout << "\nPlease specify how many trolls will be on the screen: ";
    std::cin >> numTrolls;
    for (size_t i = 0; i < numTrolls; i++) {
        size_t colourCode = 0;
        std::cout << "\nPlease specify the colour of orc " << i + 1
                  << ": Either '1' for blue, '2' for green or '3' for red. ";
        std::cin >> colourCode;
        std::shared_ptr<Character> pTroll = m_pCf->MakeCharacter(CharacterTypes::TrollType);

        SDL_Color colour = returnColourFromCode(colourCode);
        pTroll->setColour(colour);
        m_characters.push_back(std::move(pTroll));
    }
}

} // namespace patterns