#include "ros/ros.h"
#include "ros/package.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "string"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation_node");

    std::string map_path = ros::package::getPath("env_sim") + "/models/bigworld/meshes/floorplan.png";
    cv::Mat map = cv::imread(map_path);
    
    cv::imshow("Kage", map);

    return 0;
}
