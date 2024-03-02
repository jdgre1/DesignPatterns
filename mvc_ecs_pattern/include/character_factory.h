#pragma once

#ifndef CHARACTERFACTORY_H
#define CHARACTERFACTORY_H

#include <orc.h>
#include <troll.h>

#include <cassert>
#include <iostream>
#include <stdint.h>
#include <stdio.h>

namespace patterns
{

class CharacterFactory
{
public:
    // Delete copy constructor
    CharacterFactory(const CharacterFactory& obj) = delete;
    // /** * Singletons should not be assignable. */
    void operator=(const CharacterFactory&) = delete;

    static CharacterFactory* GetInstance();

    virtual Character* FactoryMethod() = 0;
    virtual Character* Character() = 0;

    void CreateCharacter(){
        Character* character = this->FactoryMethod();
    }

private:
    CharacterFactory();

    static CharacterFactory* instancePtr;
};


class OrcFactory : public CharacterFactory
{
    Character* FactoryMethod() override {
        return new Orc();
    }

};


class TrollFactory : public CharacterFactory
{
    Character* FactoryMethod() override {
        return new Troll();
    }

};


} // namespace patterns
#endif // CHARACTERFACTORY_H
