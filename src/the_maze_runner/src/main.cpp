/****
 *  Creator: Alexander Infantino
 *  Project: Mobile Robotics II Final Project
 *  Desc:    This instantiates all of the required classes and contains the main
 *           ROS loop within it.
 ****/

#include "ros/ros.h"
#include "../include/the_maze_runner/driver.hpp"
#include "../include/the_maze_runner/map.hpp"

int main(int argc, char ** argv)
{
    // ROS instantiations
    ros::init(argc, argv, "maze_runner_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    // Custom instantiations
    MazeMap map(&nh);
    Driver driver = Driver(&nh);

    // Main ROS driver
    while(ros::ok())
    {
        ros::spinOnce();

        map.UpdateMapWithRobotPosition();

        driver.DriveRobot(map.GetNextAngularMovement(), map.GetNextLinearMovement());
        
        loop_rate.sleep();
    }
}
