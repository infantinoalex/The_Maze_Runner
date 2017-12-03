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
}

void MazeMap::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    this->slamMap = *msg;

    if (this->uninitialized)
    {
        this->x_start = (int)this->slamMap.info.origin.position.x;
        this->y_start = (int)this->slamMap.info.origin.position.y;

        this->uninitialized = false;
    }
}

void MazeMap::UpdateMapWithRobotPosition()
{
    try
    {
        this-> _tfListener.lookupTransform("/map", "/base_link", ros::Time(0), this->transform);
    }
    catch (tf::TransformException &ex)
    {
        return;
    }

    int x = this->transform.getOrigin().x() * 10;
    int y = this->transform.getOrigin().y() * 10;

    std::cout << "X: " << x << "\tY: " << y << std::endl;

    for (int i = 9; i > 0; --i)
    {
        this->previousTenRobotLocations[i] = this->previousTenRobotLocations[i - 1];
    }

    std::cout << "Created Vectors" << std::endl;

    std::vector<int> newPoint;
    newPoint.push_back(y);
    newPoint.push_back(x);
    this->previousTenRobotLocations[0] = newPoint;

    std::cout << "Attempting to change slam map" << std::endl;

    for (int i = 0; i < 10; ++i)
    {
        int foundX = this->previousTenRobotLocations[i][0];
        int foundY = this->previousTenRobotLocations[i][1];

        std::cout << "adding to positon " << foundX + foundY << std::endl;
    }

    this->_mapPublisher.publish(this->slamMap);
}

double MazeMap::GetNextAngularMovement()
{
    //if (this->_pathFollower.IsFollowingPath)
    //{
        //return this->_pathFollower.CalculateNextAngularMovement();
    //}

    return 0;
}

double MazeMap::GetNextLinearMovement()
{
    //if (this->_pathFollower.IsFollowingPath)
    //{
        //return this->_pathFollower.CalculateNextLinearMovement();
    //}

    return 0;
}
