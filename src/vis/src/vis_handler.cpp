#include "vis_handler.h"
#include <opencv2/core/core.hpp>

using namespace cv;

VisHandler::VisHandler()
{
    camera_sub =
        n.subscribe("/robot/image", 1, &VisHandler::camera_callback, this);
}

void VisHandler::camera_callback(const sensor_msgs::ImageConstPtr call_img)
{
    std::size_t width = call_img->width;
    std::size_t height = call_img->height;
    char *data = call_img->data.begin();
    Mat img = Mat(int(height), int(width), CV_8UC3, data);

    imshow("Test", img);
}

VisHandler::~VisHandler()
{
}
