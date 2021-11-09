#include "vis_handler.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

VisHandler::VisHandler()
{
    camera_sub =
        n.subscribe("/robot/image_raw", 1, &VisHandler::camera_callback, this);
}

void VisHandler::camera_callback(const sensor_msgs::ImageConstPtr call_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(call_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("Test", cv_ptr->image);

    waitKey(100);
}

VisHandler::~VisHandler()
{
}
