/****
 *  Creator: Alexander Infantino
 *  Project: Mobile Robotics II Final Project
 *  Desc:    This instantiates all of the required classes and contains the main
 *           ROS loop within it.
 ****/

#include "ros/ros.h"
#include "../include/the_maze_runner/driver.hpp"

int main(int argc, char ** argv)
{
    // ROS instantiations
    ros::init(argc, argv, "maze_runner");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    // Custom instantiations
    Driver driver = Driver(&nh);
    

    // Main ROS driver
    while(ros::ok())
    {
        ros::spinOnce();

        driver.DriveRobot();

        loop_rate.sleep();
    }
}
