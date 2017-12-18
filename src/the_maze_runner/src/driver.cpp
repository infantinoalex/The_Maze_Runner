#include "../include/the_maze_runner/driver.hpp"
#include <iostream>

// Constructor
//      Initialized:
//          LaserScan subscriber
//          Movement publisher
//          Movement subscriber
//              redirects anything publish to /cmd_vel -> /robot/cmd_vel
Driver::Driver(ros::NodeHandle* nh)
{
    ROS_INFO("Initialized Driver");

    this->_laserSubscriber = nh->subscribe("/robot/base_scan", 1, &Driver::LaserScanCallback, this);
    this->_movementPublisher = nh->advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    this->_movementSubscriber = nh->subscribe("/cmd_vel", 1, &Driver::TwistCallback, this);

    this->numberOfLasers = 0;
    this->laserLimit = 1.5;
}

// Drives the robot around the map using a right wall following algorithm
void Driver::DriveRobot()
{
    // Ensures that the robot does not hit the wall
    if (this->rightMostSensor < .5)
    {
        this->movementMsg.angular.z = .5;
        this->movementMsg.linear.x = 0;
    }
    else if (this->rightMostSensor >= this->laserLimit && this->middleSensor >= this->laserLimit)
    {
        this->movementMsg.angular.z = -.5;
        this->movementMsg.linear.x = 0.5;
    }
    else if (this->rightMostSensor >= this->laserLimit && this->middleSensor < this->laserLimit)
    {
        this->movementMsg.angular.z = .5;
        this->movementMsg.linear.x = 0;
    }
    else if (this->rightMostSensor < this->laserLimit && this->middleSensor >= this->laserLimit)
    {
        this->movementMsg.angular.z = 0;
        this->movementMsg.linear.x = 0.5;
    } 
    else
    {
        this->movementMsg.angular.z = .5;
        this->movementMsg.linear.x = 0;
    }

    this->_movementPublisher.publish(this->movementMsg);
}

// Updates the LaserScan values when it receives laser data
void Driver::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("In Laser Scan Callback");

    if (this->numberOfLasers == 0)
    {
        try
        {
            this->numberOfLasers = msg->ranges.size();
        }
        catch (int exception)
        {
            this->numberOfLasers = 0;
            return;
        }
    }

    this->rightMostSensor = msg->ranges[0];
    this->middleSensor = msg->ranges[(this->numberOfLasers / 2) - 1];
}

// Gets the cmd_vel data to republish
void Driver::TwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->_movementPublisher.publish(*msg);
}
