#include "../include/the_maze_runner/follower.hpp"

Follower::Follower(ros::NodeHandle *nh)
{
    this->_navigationPublisher = nh->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    this->hasPublishedGoal = false;
}

void Follower::SetMazeStartCoordinates(int xCoordinate, int yCoordinate)
{
    this->mazeStartX = xCoordinate;
    this->mazeStartY = yCoordinate;
}

int Follower::GetMazeStartX()
{
    return this->mazeStartX;
}

int Follower::GetMazeStartY()
{
    return this->mazeStartY;
}

void Follower::SolveMazeAndFollow()
{
    if (this->hasPublishedGoal)
    {
        return;
    }

    geometry_msgs::PoseStamped moveGoal;

    moveGoal.pose.position.x = this->mazeStartX;
    moveGoal.pose.position.y = this->mazeStartY;

    this->_navigationPublisher.publish(moveGoal);

    this->hasPublishedGoal = true;
}
