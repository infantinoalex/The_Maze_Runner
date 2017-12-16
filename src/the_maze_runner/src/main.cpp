/****
 *  Creator: Alexander Infantino
 *  Project: Mobile Robotics II Final Project
 *  Desc:    This instantiates all of the required classes and contains the main
 *           ROS loop within it.
 ****/

#include "ros/ros.h"
#include <termios.h>
#include "../include/the_maze_runner/driver.hpp"
#include "../include/the_maze_runner/map.hpp"
#include "../include/the_maze_runner/follower.hpp"

// Non blocking input from stdin
int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int c = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

int main(int argc, char ** argv)
{
    // ROS instantiations
    ros::init(argc, argv, "maze_runner_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    // Custom instantiations
    MazeMap map(&nh);
    Follower follower(&nh);
    Driver driver(&nh);
    
    ROS_INFO("Initialized");

    // Main ROS driver
    while(ros::ok())
    {
        ros::spinOnce();

        int c = getch();

        if (c == ' ')
        {
            ROS_INFO("Read Space Input");

            if (!map.uninitialized)
            {
                map.mapComplete = true;
                ROS_INFO("Map Complete. Finding Path Out");
            }
        }

        map.UpdateMapWithRobotPosition();

        if (!map.mapComplete)
        {
            driver.DriveRobot();

            if (!map.uninitialized)
            {
                follower.SetMazeStartCoordinates(map.startX, map.startY);
            }
        }
        else
        {
            follower.SolveMazeAndFollow();
        }

        loop_rate.sleep();
    }
}
