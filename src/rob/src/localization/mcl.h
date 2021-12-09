#pragma once
#include "ros/ros.h"
#include "particle.h"
#include "vector"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "math.h"
#include <stdlib.h>
#include <random>

#define REAL_WIDTH 84.67
#define REAL_HEIGHT 56.45

class Particle;

class MCL
{
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    cv::Mat* map;
    std::vector<Particle> particles;
    int particle_amount = 10000;
    float map2real_c;

public:
    MCL(cv::Mat* map);
    void randomize_particles();
    void visualize();
    void lidar_callback(const sensor_msgs::LaserScanConstPtr& scan);
    void resample(std::vector<double> weights);

    float map2real(float);
    int real2map(float);
    int real2map_x(float);
    int real2map_y(float);
    float map2real_x(int);
    float map2real_y(int);
    ~MCL();
};
