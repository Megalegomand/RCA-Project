#include "ros/ros.h"
#include "vision_handler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vis_node");

    VisHandler vh = VisHandler();

    ros::spin();
}
