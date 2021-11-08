#include "fuzzy_control.h"

FuzzyControl::FuzzyControl()
{
    lidar_sub = n.subscribe("/robot/laser/scan", 1, 
                            &FuzzyControl::lidar_callback, this);

    movement_pub = n.advertise<geometry_msgs::Twist>("/robot/control", 100);
}

void FuzzyControl::lidar_callback(const sensor_msgs::LaserScan::ConstPtr scan)
{
    // Default to center angle and max range
    float min_angle = (scan->angle_min + scan->angle_max) / 2;
    float min_range = scan->range_max;

    for (int i = 0; i < scan->ranges.size(); i++)
    {
        float range = scan->ranges[i];
        if (range < min_range && range > scan->range_min)
        {
            min_range = range;
            min_angle = scan->angle_min + scan->angle_increment * i;
        }
    }

    ROS_INFO("Angle %5f : Range %5f", min_angle, min_range);
}

FuzzyControl::~FuzzyControl()
{
}
