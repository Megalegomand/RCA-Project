#include "ros/ros.h"
#include "particle.h"
#include "vector"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "math.h"

class MCL
{
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    cv::Mat* map;
    std::vector<Particle> particles;
    int particle_amount = 1000;
public:
    MCL(cv::Mat* map);
    void randomize_particles();
    void visualize();
    void lidar_callback(const sensor_msgs::LaserScanConstPtr& scan);
    ~MCL();
};
