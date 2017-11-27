/****
 *  Creator: Alexander Infantino
 *  Project: Mobile Robotics II Final Project
 *  Desc:    This instantiates all of the required classes and contains the main
 *           ROS loop within it.
 ****/

#include "ros/ros.h"

int main(int argc, char ** argv)
{
    // ROS instantiations
    ros::init(argc, argv, "maze_runner");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    // Custom instantiations
    

    // Main ROS driver
    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
}
