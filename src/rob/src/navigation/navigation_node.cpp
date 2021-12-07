#include "ros/ros.h"
#include "ros/package.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "string"
#include "rrt.h"
#include "rrt_point.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation_node");

    std::string map_path = ros::package::getPath("env_sim") + "/models/bigworld/meshes/floor_plan.png";
    cv::Mat map = cv::imread(map_path);

    cv::resize(map, map, cv::Size(), 4.0f, 4.0f, cv::INTER_NEAREST);

    ROS_INFO("Map size: (%i, %i)", map.cols, map.rows);

    RRTPoint start(10, 10);
    RRT rrt(&map, start);
    RRTPoint end(50, 50);
    rrt.build(end, true);

    cv::imshow("Kage", map);
    cv::waitKey();

    return 0;
}
