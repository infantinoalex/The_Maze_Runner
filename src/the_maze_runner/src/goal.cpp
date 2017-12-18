#include "ros/ros.h"
#include "the_maze_runner/goal.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Uses the move_base action client to send goals. Code is taken from the ROS tutorials on writing a simple move_base client

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

the_maze_runner::goal goal;

// The custom message created to receive goal data to publish to the move_base client
void GoalCallback(const the_maze_runner::goal::ConstPtr& msg)
{
    ROS_INFO("Received Goal");
    goal = *msg;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;

    ros::Subscriber goalSub = nh.subscribe("my_goal", 1, GoalCallback);

    ros::Rate loop_rate(10);

    goal.ready = false;

    MoveBaseClient ac("move_base", true);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        if (goal.ready)
        {
            ROS_INFO("Got Goal");
            break;
        }
    }

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }


    move_base_msgs::MoveBaseGoal new_goal;

    new_goal.target_pose.header.frame_id = "/map";
    new_goal.target_pose.header.stamp = ros::Time();

    new_goal.target_pose.pose.position.x = goal.x;
    new_goal.target_pose.pose.position.y = goal.y;

    new_goal.target_pose.pose.orientation.w = 1.0;
    
    do
    {  
        ROS_INFO("Sending Goal");
        ac.sendGoal(new_goal);

        ac.waitForResult();
    } while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

    ROS_INFO("Reached Goal");
}
