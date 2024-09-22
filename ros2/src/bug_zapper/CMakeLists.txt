cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

Project(bug_zapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(OpenCV REQUIRED)

# find_package(MVC_ECS_PATTERN REQUIRED)

include_directories(bug_zapper/include)
include_directories(sim)
include_directories(${rclcpp_INCLUDE_DIRS}
					${rclcpp_lifecycle_INCLUDE_DIRS} 
					${OpenCV_INCLUDE_DIRS}
					"${CMAKE_CURRENT_SOURCE_DIR}/../../../mvc_ecs_pattern/include"
					)

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
          
)   

add_executable(BUG_SIM
    sim/bug.h
    sim/bug_factory.h
    sim/bug_sim.h
    sim/bug_sim.cpp
    sim/main.cpp
)

ament_target_dependencies(BUG_ZAP rclcpp)
ament_target_dependencies(BUG_SIM rclcpp)


target_link_libraries(BUG_SIM 
					${WEBP_LIBRARY}
					${ILMIMF_LIBRARY}
					${OpenCV_LIBS}   
					${rclcpp_LIBRARIES} 
					${rclcpp_lifecycle_LIBRARIES}
					)


install(TARGETS
	BUG_ZAP
	BUG_SIM
  	DESTINATION lib/${PROJECT_NAME})

ament_package()

