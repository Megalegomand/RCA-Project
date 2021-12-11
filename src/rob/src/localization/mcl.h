#pragma once
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "particle.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "vector"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>
#include <stdlib.h>
#include "tf/transform_datatypes.h"

#define REAL_WIDTH 84.67
#define REAL_HEIGHT 56.45
#define POS_STD 0.5
#define ANGLE_STD 0.5

class Particle;

class MCL
{
  private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::LaserScan, nav_msgs::Odometry>
        ParticleSyncPolicy;
    typedef message_filters::Synchronizer<ParticleSyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    cv::Mat *map;
    std::vector<Particle> particles;
    int particle_amount = 10000;
    float map2real_c;

    float prev_x = 0;
    float prev_y = 0;
    float prev_angle = 0;

  public:
    MCL(cv::Mat *map);
    void randomize_particles();
    void visualize();
    void callback(const sensor_msgs::LaserScanConstPtr &scan,
                  const nav_msgs::OdometryConstPtr &odom);
    void resample(std::vector<double> weights);

    float map2real(float);
    int real2map(float);
    int real2map_x(float);
    int real2map_y(float);
    float map2real_x(int);
    float map2real_y(int);
    ~MCL();
};
