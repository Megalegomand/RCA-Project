#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class VisHandler
{
private:
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
public:
    VisHandler();

    void camera_callback(const sensor_msgs::ImageConstPtr call_img);

    ~VisHandler();
};
