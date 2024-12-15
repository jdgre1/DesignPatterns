cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

Project(bug_zapper_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(MVC_ECS_PATTERN REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/FireCommand.msg
    DEPENDENCIES std_msgs
)
 

ament_export_dependencies(rosidl_default_runtime)

ament_package()

