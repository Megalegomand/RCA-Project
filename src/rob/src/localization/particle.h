#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "vector"
#include "math.h"
#include "assert.h"
#include "mcl.h"

#define MARK_DISTANCE 10
#define LASER_STD 1.0
#define LASER_DIVISIONS 10

class MCL;

class Particle
{
private:
    float x;
    float y;
    float angle;
    cv::Mat* map;
    MCL* parent;
    float weight;

    float wall_distance(float angle);
public:
    double norm_weight;
    Particle(float x, float y, float angle, cv::Mat* map, MCL* parent);
    void mark(cv::Mat* vis_map);
    double get_likelihood(const sensor_msgs::LaserScanConstPtr& scan);
    ~Particle();
};
