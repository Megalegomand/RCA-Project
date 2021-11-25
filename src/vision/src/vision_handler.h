#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class VisHandler
{
  private:
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
    ros::Subscriber camera_info_sub;
    std::vector<double> cam_matrix = {277.19135641132203,
                                      0.0,
                                      160.5,
                                      0.0,
                                      277.19135641132203,
                                      120.5,
                                      0.0,
                                      0.0,
                                      1.0};
    std::vector<double> dist_vec = {-0.25, 0.12, -0.00028, -0.00005, 0.0};

  public:
    VisHandler();

    void camera_callback(const sensor_msgs::ImageConstPtr &call_img);
    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &cam_info);

    ~VisHandler();
};
