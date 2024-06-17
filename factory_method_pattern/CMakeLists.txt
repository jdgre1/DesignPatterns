cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 17)
Project(COFFEE_FACTORY_PATTERN)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(coffee_factory
			main.cpp
			coffee_factory.h
            coffee.h
            coffee_variants.h
)   
