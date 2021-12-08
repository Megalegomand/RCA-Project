#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "sensor_msgs/LaserScan.h"

#define MARK_DISTANCE 3

class Particle
{
private:
    float x;
    float y;
    float angle;

    float val_from_gazebo(float val);
public:
    Particle(float x, float y, float angle);
    void mark(cv::Mat* map);
    float sensor_update(const sensor_msgs::LaserScanConstPtr scan);
    ~Particle();
};
