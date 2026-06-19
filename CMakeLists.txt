cmake_minimum_required(VERSION 3.10)
project(theta_star_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES theta_star_lib kinodynamic_optimizer_lib
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs visualization_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS Threads
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(theta_star_lib STATIC
  src/theta_star.cpp
)

target_include_directories(theta_star_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(theta_star_lib PUBLIC
  Threads::Threads
  ${catkin_LIBRARIES}
)

add_library(kinodynamic_optimizer_lib STATIC
  src/kinodynamic_optimizer.cpp
)

target_include_directories(kinodynamic_optimizer_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(kinodynamic_optimizer_lib PUBLIC
  theta_star_lib
)


# Navigation node with laser support
add_executable(theta_star_nav_node
  src/theta_star_nav_node.cpp
)

target_link_libraries(theta_star_nav_node
  PRIVATE theta_star_lib kinodynamic_optimizer_lib
)

target_include_directories(theta_star_nav_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(theta_star_nav_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch rviz config map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
