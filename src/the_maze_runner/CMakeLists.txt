cmake_minimum_required(VERSION 2.8.3)
project(the_maze_runner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  input.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_runner src/main.cpp
                           src/driver.cpp
                           src/map.cpp
                           src/follower.cpp)

add_executable(key_listener src/spacebar.cpp)

target_link_libraries(maze_runner ${catkin_LIBRARIES})
target_link_libraries(key_listener ${catkin_LIBRARIES})
add_dependencies(maze_runner ${catkin_LIBRARIES})
add_dependencies(key_listener ${catkin_LIBRARIES})
