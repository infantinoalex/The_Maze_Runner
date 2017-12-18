#include "../include/the_maze_runner/map.hpp"

MazeMap::MazeMap(ros::NodeHandle* nh)
{
    this->_mapSubscriber = nh->subscribe("/map", 1, &MazeMap::MapCallback, this);
    this->_mapPublisher = nh->advertise<nav_msgs::OccupancyGrid>("/displaymap", 1);

    for (int i = 0; i < 10; ++i)
    {
        std::vector<int> innerVector;
        innerVector.push_back(0);
        innerVector.push_back(0);
        this->previousTenRobotLocations.push_back(innerVector);
    }

    this->uninitialized = true;
    this->initializeCount = 0;
    this->mapComplete = false;
}

void MazeMap::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (this->initializeCount < 2)
    {
        this->initializeCount++;
    }

    this->slamMap = *msg;
}

float MazeMap::GetStartX()
{
    return this->startX;
}

float MazeMap::GetStartY()
{
    return this->startY;
}

void MazeMap::UpdateMapWithRobotPosition()
{
    if (this->initializeCount < 1)
    {   
        return;
    }

    if (!this->mapComplete)
    {
        try
        {
            // Used to get the position of the robot in the map
            this->_tfListener.lookupTransform("/map", "/base_link", ros::Time(0), this->transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_INFO("Transform Exception caught. Not calculated robots position in map.");
            return;
        }

        // Gets the x value of the index of the array
        int x = 
            (this->transform.getOrigin().x() - this->slamMap.info.origin.position.x) / this->slamMap.info.resolution;
    
        // Gets the y value of the index of the array
        int y = 
            (this->transform.getOrigin().y() - this->slamMap.info.origin.position.y) / this->slamMap.info.resolution;

        // Sets the origin of the robot in the map
        if (this->uninitialized)
        {
            this->startX = this->transform.getOrigin().x() - this->slamMap.info.origin.position.x;
            this->startY = this->transform.getOrigin().y() - this->slamMap.info.origin.position.y;

            this->uninitialized = false;
        }

        // Used to print the last 10 locations of the robot in the map
        for (int i = 9; i > 0; --i)
        {
            this->previousTenRobotLocations[i] = this->previousTenRobotLocations[i - 1];
        }

        std::vector<int> newPoint;
        newPoint.push_back(y);
        newPoint.push_back(x);
        this->previousTenRobotLocations[0] = newPoint;

        for (int i = 0; i < 10; ++i)
        {
            int foundX = this->previousTenRobotLocations[i][0];
            int foundY = this->previousTenRobotLocations[i][1];

            this->slamMap.data[foundX * this->slamMap.info.width + foundY] = -127;
        }
    }

    // Visually Publishes the Origin of the Map in a Orange Cross
    for (int i = -5; i < 5; ++i)
    {
        this->slamMap.data[(this->startX) * this->slamMap.info.width + (this->startY + i)] = -50;
        this->slamMap.data[(this->startX + i) * this->slamMap.info.width + (this->startY - i)] = -50;
        this->slamMap.data[(this->startX + i) * this->slamMap.info.width + (this->startY)] = -50;
        this->slamMap.data[(this->startX + i) * this->slamMap.info.width + (this->startY + i)] = -50;
    }
    
    this->_mapPublisher.publish(this->slamMap);
}
