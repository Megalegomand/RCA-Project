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
#include "vector"
#include <string>

class VisHandler
{
  private:
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
    ros::Subscriber camera_info_sub;
    cv::Mat camera_matrix;
    cv::Mat distortion_vector;
    

  public:
    VisHandler();

    void camera_callback(const sensor_msgs::ImageConstPtr &call_img);

    ~VisHandler();
};
