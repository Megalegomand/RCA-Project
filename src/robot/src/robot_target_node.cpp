#include "ros/ros.h"
#include "robot/GetPoint.h"
#include "geometry_msgs/Point.h"
#include <mutex>

std::mutex target_mutex;
geometry_msgs::Point target;

bool get_point_callback(robot::GetPoint::Request& req, robot::GetPoint::Response& res)
{
    target_mutex.lock();
    res.point = target;
    target_mutex.unlock();
    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_target_node");

    ros::NodeHandle n;



    return 0;
}
