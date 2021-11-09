#include "ros/ros.h"
#include "vis_handler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vis_node");

    VisHandler vh;

    ros::spin();
}
