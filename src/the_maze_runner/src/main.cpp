/****
 *  Creator: Alexander Infantino
 *  Project: Mobile Robotics II Final Project
 *  Desc:    This instantiates all of the required classes and contains the main
 *           ROS loop within it.
 ****/

#include "ros/ros.h"
#include "the_maze_runner/input.h"
#include "../include/the_maze_runner/driver.hpp"
#include "../include/the_maze_runner/map.hpp"
#include "../include/the_maze_runner/follower.hpp"

void SpacebarCallback(const the_maze_runner::input::ConstPtr& msg)
{
    isSpacePressed = msg->spacePressed;
}

the_maze_runner::input isSpacePressed;

int main(int argc, char ** argv)
{
    // ROS instantiations
    ros::init(argc, argv, "maze_runner_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    // Custom instantiations
    ros::Subscriber spaceBarSub = nh.subscribe("space_input", 1, SpacebarCallback);

    MazeMap map(&nh);
    Follower follower(&nh);
    Driver driver(&nh);
    
    ROS_INFO("Initialized");

    // Main ROS driver
    while(ros::ok())
    {
        ros::spinOnce();

        if (false)
        {
            ROS_INFO("Read Space Input");

            if (!map.uninitialized)
            {
                map.mapComplete = true;
                ROS_INFO("Map Complete. Finding Path Out");
            }
        }

        map.UpdateMapWithRobotPosition();

        if (!map.uninitialized)
        {
            if (!map.mapComplete)
            {
                ROS_INFO("Right Wall Follow");
                driver.DriveRobot();

                follower.SetMazeStartCoordinates(map.startX, map.startY);
            }
            else
            {
                ROS_INFO("Following Path");
                follower.SolveMazeAndFollow();
            }
        }
        else
        {
            ROS_INFO("Map Uninitialized");
        }

        loop_rate.sleep();
    }
}
