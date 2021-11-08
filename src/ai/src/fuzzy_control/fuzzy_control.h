#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <algorithm>

using namespace std;

class FuzzyControl
{
private:
    ros::NodeHandle n;
    ros::Subscriber lidar_sub;
    ros::Publisher movement_pub;
public:
    FuzzyControl(/* args */);

    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr scan);

    ~FuzzyControl();
};
