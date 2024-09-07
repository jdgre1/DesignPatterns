cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 17)
Project(BUG_ZAPPER)

include_directories(bug_zapper/include)
include_directories(sim)

add_executable(BUG_ZAP
			bug_zapper/src/main.cpp
			bug_zapper/src/bug_tracker.cpp
            bug_zapper/src/bug_zapper.cpp
            bug_zapper/src/camera.cpp
            bug_zapper/src/controller.cpp
            bug_zapper/src/fire_controller.cpp
            bug_zapper/src/map.cpp
            bug_zapper/src/position.cpp
            bug_zapper/src/utils.cpp
            bug_zapper/src/viewer.cpp
            sim/bug_sim.cpp
)   

target_link_libraries(BUG_ZAP)