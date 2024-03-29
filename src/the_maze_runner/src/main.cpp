/****
 *  Creator: Alexander Infantino
 *  Project: Mobile Robotics II Final Project
 *  Desc:    This instantiates all of the required classes and contains the main
 *           ROS loop within it.
 ****/

#include "ros/ros.h"
#include "the_maze_runner/input.h"
#include "the_maze_runner/goal.h"
#include "../include/the_maze_runner/driver.hpp"
#include "../include/the_maze_runner/map.hpp"
#include "../include/the_maze_runner/follower.hpp"

bool spacePressed;
void SpacebarCallback(const the_maze_runner::input::ConstPtr& msg)
{
    spacePressed = msg->spacePressed;
}

int main(int argc, char ** argv)
{
    // ROS instantiations
    ros::init(argc, argv, "maze_runner_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    // Custom instantiations
    ros::Subscriber spaceBarSub = nh.subscribe("space_input", 1, SpacebarCallback);
    ros::Publisher goalPub = nh.advertise<the_maze_runner::goal>("my_goal", 1);

    Follower follower(&nh);
    MazeMap map(&nh);
    Driver driver(&nh);
    
    // Main ROS driver
    while(ros::ok())
    {
        ros::spinOnce();

        map.UpdateMapWithRobotPosition();
        
        if (spacePressed)
        {
            if (!map.uninitialized)
            {
                map.mapComplete = true;
            }
        }
        
        if (!map.uninitialized)
        {
            if (!map.mapComplete)
            {
                driver.DriveRobot();

                follower.SetMazeStartCoordinates(map.GetStartX(), map.GetStartY());
            }
            else
            {
                //follower.SolveMazeAndFollow();
                the_maze_runner::goal myGoal;
                myGoal.x = map.GetStartX();
                myGoal.y = map.GetStartY();
                myGoal.ready = true;
                goalPub.publish(myGoal);
            }
        }

        loop_rate.sleep();
    }
}
