#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "vector"
#include "math.h"
#include "assert.h"

#define MARK_DISTANCE 3
#define LASER_STD 1.0
#define LASER_DIVISIONS 10
#define REAL_WIDTH 84.67
#define REAL_HEIGHT 56.45

class Particle
{
private:
    float x;
    float y;
    float angle;
    cv::Mat* map;
    float map2real;
    float weight;

    float wall_distance(float angle);
public:
    Particle(float x, float y, float angle, cv::Mat* map);
    void mark(cv::Mat* vis_map);
    double get_likelihood(const sensor_msgs::LaserScanConstPtr& scan);
    ~Particle();
};
