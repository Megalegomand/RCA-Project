#include "ros/ros.h"
#include "sensor_msgs/Image.h"

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
