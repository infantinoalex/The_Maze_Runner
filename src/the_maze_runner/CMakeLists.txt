cmake_minimum_required(VERSION 2.8.3)
project(the_maze_runner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_runner src/main.cpp
                           src/driver.cpp
                           src/map.cpp
                           src/path.cpp
                           src/bot.cpp
                           src/follower.cpp)

target_link_libraries(maze_runner ${catkin_LIBRARIES})
add_dependencies(maze_runner ${catkin_LIBRARIES})
