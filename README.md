# DEPENDENCIES:
1. gmapping
2. slam_mapping
3. move_base

# Running Code
1. roslaunch the_maze_runner easy_maze.launch
2. roslaunch the_maze_runner medium_maze.launch
3. roslaunch the_maze_runner hard_maze.launch

These must be run seperately

1. rosrun the_maze_runner key_listener
    listens for spacebar input
2. rosrun the_maze_runner goal_listener
    listens for a goal to be published from the main program and published to move_base
