#include "ros/ros.h"
#include <termios.h>

// This code taken from ROS ANSWERS
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
    ros::init(argc, argv, "space_bar_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Publisher spacePublisher = nh.advertise<the_maze_runner::input>("spacePressed", 1);

    while (ros::ok())
    {
        int c = getch();
        if (c == ' ')
        {
            ROS_INFO("Space Bar Pressed");
            the_maze_runner::input spacePressed.spacePressed = true;

            spacePublisher.publish(spacePressed);
        }
    }
}
