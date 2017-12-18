#include "../include/the_maze_runner/follower.hpp"

// Initializes the publisher to publish to topic move_base_simple/goal
Follower::Follower(ros::NodeHandle *nh)
{
    this->_navigationPublisher = nh->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
}

// Sets the start coordinates of the maze for further usage
void Follower::SetMazeStartCoordinates(float xCoordinate, float yCoordinate)
{
    this->mazeStartX = xCoordinate;
    this->mazeStartY = yCoordinate;
}

// Used to publish a simple goal to the move_base client
void Follower::SolveMazeAndFollow()
{
    geometry_msgs::PoseStamped moveGoal;

    moveGoal.pose.position.x = this->mazeStartX;
    moveGoal.pose.position.y = this->mazeStartY;
    moveGoal.pose.orientation.w = 1.0;
    moveGoal.header.frame_id = "/map";
    moveGoal.header.stamp = ros::Time();

    this->_navigationPublisher.publish(moveGoal);

    this->hasPublishedGoal = true;
}
