#pragma once

#ifndef CHARACTERFACTORY_H
#define CHARACTERFACTORY_H

#include <orc.h>
#include <troll.h>

#include <memory>

#include <cassert>
#include <iostream>
#include <stdint.h>
#include <stdio.h>

namespace patterns
{
    enum class CharacterTypes
    {
         OrcType,
         TrollType
    };

class CharacterFactoryBase
{
public:
    virtual std::unique_ptr<Character> MakeCharacter(CharacterTypes characterType) = 0;

};

class CharacterFactory : CharacterFactoryBase
{
    public:
        std::unique_ptr<Character> MakeCharacter(CharacterTypes characterType) override
        {
            switch(characterType)
            {
                case CharacterTypes::OrcType:
                    return std::unique_ptr<Character>(std::make_unique<Orc>());

                case CharacterTypes::TrollType:
                    return std::unique_ptr<Character>(std::make_unique<Troll>());

                default:
                {
                    std::cout << "\nCharacter type does not exist in factory";
                    return nullptr;
                }
            }
        }

};
// class OrcFactory : public CharacterFactory
// {

//      std::unique_ptr<Character> FactoryMethod() override {
//         return std::make_unique<Orc>();
//     }

// };


// class TrollFactory : public CharacterFactory
// {
//     std::unique_ptr<Character> FactoryMethod() override {
//         return std::make_unique<Troll>();
//     }

// };


} // namespace patterns
#endif // CHARACTERFACTORY_H
