#include "../include/the_maze_runner/driver.hpp"
#include <iostream>

Driver::Driver(ros::NodeHandle *nh)
{
    this->_nodeHandle = *nh;
    this->_laserSubscriber = this->_nodeHandle.subscribe("/robot/base_scan", 1, &Driver::LaserScanCallback, this);
    this->_movementPublisher = this->_nodeHandle.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    
    this->numberOfLasers = 0;
    this->mazeSolved = false;
    this->laserLimit = 1.5;
}

void Driver::DriveRobot(double newAngular, double newLinear)
{
    if (!this->mazeSolved)
    {
        this->RightWallFollow();
    }
    else
    {
        this->movementMsg.angular.z = newAngular;
        this->movementMsg.linear.x = newLinear;
    }

    this->_movementPublisher.publish(this->movementMsg);
}

void Driver::RightWallFollow()
{
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
}

void Driver::FollowPath()
{

}

void Driver::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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
