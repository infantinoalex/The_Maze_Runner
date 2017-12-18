# DEPENDENCIES:
1. gmapping
2. slam_mapping
3. move_base

# Running Code
1. roslaunch the_maze_runner easy_maze.launch
2. roslaunch the_maze_runner medium_maze.launch
3. roslaunch the_maze_runner hard_maze.launch
    Does not exist, could not make hard maze

These must be run seperately

1. rosrun the_maze_runner key_listener
    listens for spacebar input
2. rosrun the_maze_runner goal_listener
    listens for a goal to be published from the main program and published to move_base
3. rosrun rviz rviz
    to view the map and plan it makes

# How it works

rosrun the key listener and goal listener in separate terminals
Select what maze you want, either the easy or hard and launch it in a seperate terminal.

The robot should begin moving around and mapping out its environment.

To view the custom map with the origin of the robot highlighted and its path, 
view the /displaymap topic on rviz

When you want the robot to try to find its way back to the start, make sure the key_listener
terminal is active and hit the space bar.

The robot should now use move_base to plan its way back to the beginning.

# Known Issues

1. The gmapping does not produce a good result no matter how much I change around the parameters.
2. Path planning does not always work. Sometimes the plans try to go in unkown space instead of known.
3. The plan may look good, but as the robot moves around the map messes up messing up the plan.
