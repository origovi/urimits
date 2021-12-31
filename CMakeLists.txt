cmake_minimum_required(VERSION 3.0.2)
project(urimits)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs dv_msgs dynamic_reconfigure)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp)
add_library(${PROJECT_NAME}_libs
  src/Urimits.cpp
  include/Urimits.hh
  include/Pos.hh
  include/Trace.hh
)

add_dependencies(${PROJECT_NAME} dv_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_libs)