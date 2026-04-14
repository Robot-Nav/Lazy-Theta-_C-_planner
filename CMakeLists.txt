cmake_minimum_required(VERSION 3.10)
project(ThetaStar_Planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3)

find_package(Threads REQUIRED)

add_library(theta_star_lib STATIC
  src/theta_star.cpp
)

target_include_directories(theta_star_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(theta_star_lib PUBLIC Threads::Threads)

add_executable(theta_star_demo
  src/main.cpp
)

target_link_libraries(theta_star_demo PRIVATE theta_star_lib)

target_include_directories(theta_star_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
