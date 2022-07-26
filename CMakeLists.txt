cmake_minimum_required(VERSION 3.0.2)
project(urimits)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs as_msgs sensor_msgs)
find_package(Eigen3 REQUIRED)

generate_messages(DEPENDENCIES as_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_exec src/main.cpp)
add_library(${PROJECT_NAME}
  src/Urimits.cpp
)

add_dependencies(${PROJECT_NAME}_exec as_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_exec ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)