cmake_minimum_required(VERSION 3.8)
project(cg_cpp_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(get_map_client src/get_map_client.cpp)
ament_target_dependencies(get_map_client rclcpp cg_interfaces)

install(TARGETS 
    get_map_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
