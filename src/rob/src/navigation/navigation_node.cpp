#include "ros/package.h"
#include "ros/ros.h"
#include "rrt.h"
#include "rrt_point.h"
#include "string"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

bool mouse_waiting = false;
RRTPoint mouse_coordinate(0, 0);

RRTPoint wait_mouse()
{
    mouse_waiting = true;
    while (mouse_waiting)
    {
        waitKey(10);
    }
    ROS_INFO("Mouse %i, %i", mouse_coordinate.get_x(),
             mouse_coordinate.get_y());
    return mouse_coordinate;
}

void mouse_callback(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDBLCLK)
    {
        if (mouse_waiting)
        {
            mouse_coordinate = RRTPoint(x, y);
            mouse_waiting = false;
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation_node");

    std::string map_path = ros::package::getPath("env_sim") +
                           "/models/bigworld/meshes/floor_plan.png";
    cv::Mat map = cv::imread(map_path);
    cv::namedWindow("Visual");

    cv::resize(map, map, cv::Size(), 4.0f, 4.0f, cv::INTER_NEAREST);

    cv::imshow("Visual", map);
    cv::setMouseCallback("Visual", &mouse_callback);

    ROS_INFO("Map size: (%i, %i)", map.cols, map.rows);

    RRTPoint start = wait_mouse();
    RRT rrt(&map, start);
    RRTPoint end = wait_mouse();
    rrt.connect(end, false);

    rrt.visualize();
    cv::waitKey();

    return 0;
}
