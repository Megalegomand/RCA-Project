#include "ros/ros.h"
#include "ros/package.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "localisation_node");

    std::string map_path = ros::package::getPath("env_sim") +
                           "/models/bigworld/meshes/floor_plan.png";
    cv::Mat map = cv::imread(map_path);
    cv::namedWindow("Visual");

    cv::resize(map, map, cv::Size(), 4.0f, 4.0f, cv::INTER_NEAREST);


    return 0;
}
