cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 17)
Project(MVC_ECS_PATTERN)

find_package(SDL2 REQUIRED)

include_directories(include ${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS})

add_executable(ECS_DEMO
			src/main.cpp
			src/menu.cpp
            include/character.h
            include/character_factory.h
            src/components.cpp
            src/controller.cpp
            src/player.cpp
            src/entity_manager.cpp
            src/game.cpp
            src/system.cpp
            src/utils.cpp
            src/viewer.cpp
)   

target_link_libraries(ECS_DEMO ${SDL2_LIBRARIES} ${SDL2IMAGE_LIBRARIES})