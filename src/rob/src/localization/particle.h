#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "vector"
#include "math.h"

#define MARK_DISTANCE 3
#define LASER_STD 0.03f

class Particle
{
private:
    float x;
    float y;
    float angle;
    cv::Mat* map;

    float wall_distance(float angle);
public:
    Particle(float x, float y, float angle, cv::Mat* map);
    void mark(cv::Mat* vis_map);
    float sensor_update(const sensor_msgs::LaserScanConstPtr& scan);
    ~Particle();
};
