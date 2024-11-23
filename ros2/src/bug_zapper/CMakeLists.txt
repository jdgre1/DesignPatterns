cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

Project(bug_zapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)

# find_package(MVC_ECS_PATTERN REQUIRED)

include_directories(bug_zapper/include bug_zapper/msgs)
include_directories(sim)
include_directories(${rclcpp_INCLUDE_DIRS}
					${rclcpp_lifecycle_INCLUDE_DIRS} 
					${OpenCV_INCLUDE_DIRS}
					"${CMAKE_CURRENT_SOURCE_DIR}/../../../mvc_ecs_pattern/include"
					)

add_executable(BUG_ZAP
	bug_zapper/src/main.cpp
	bug_zapper/src/bug_detector.cpp
	bug_zapper/src/bug_manager.cpp
	bug_zapper/src/bug_tracker.cpp
	bug_zapper/src/bug_zapper.cpp
	bug_zapper/src/camera.cpp
	bug_zapper/include/config.h
	bug_zapper/src/fire_controller.cpp
	bug_zapper/src/map.cpp
	bug_zapper/src/position.cpp
	bug_zapper/include/utils.h
	bug_zapper/src/viewer.cpp
	bug_zapper/msgs/bug_detection.h
          
)   

add_executable(BUG_SIM
    sim/bug.h
    sim/bug_factory.h
    sim/bug_sim.h
    sim/bug_sim.cpp
    sim/vehicle_sim.h
    sim/vehicle_sim_node.h
    sim/main.cpp
)

ament_target_dependencies(BUG_ZAP rclcpp cv_bridge geometry_msgs sensor_msgs tf2_ros)
ament_target_dependencies(BUG_SIM rclcpp cv_bridge geometry_msgs sensor_msgs tf2_ros)


target_link_libraries(BUG_SIM 
					${OpenCV_LIBS}   
					${rclcpp_LIBRARIES} 
					${rclcpp_lifecycle_LIBRARIES}
					)
target_link_libraries(BUG_ZAP 
					${OpenCV_LIBS}   
					${rclcpp_LIBRARIES} 
					${rclcpp_lifecycle_LIBRARIES}
					)
install(TARGETS
	BUG_ZAP
	BUG_SIM
  	DESTINATION lib/${PROJECT_NAME})

install(
	DIRECTORY 
		bug_zapper/launch 
		bug_zapper/config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()

