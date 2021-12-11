#include "ros/package.h"
#include "ros/ros.h"
#include "rrt.h"
#include "rrt_point.h"
#include "string"
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
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

template <typename T> struct comma_separator : std::numpunct<T>
{
    typename std::numpunct<T>::char_type do_decimal_point() const
    {
        return ',';
    }
};

template <typename T>
std::basic_ostream<T> &comma_sep(std::basic_ostream<T> &os)
{
    os.imbue(std::locale(std::locale(""), new comma_separator<T>));
    return os;
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

    // RRTPoint start = wait_mouse();
    RRTPoint start(10, 10);
    RRTPoint end(470, 310);
    RRTPoint end2(470, 10);

    for (float s = 0.01; s < 0.06; s += 0.01)
    {
        int step_size = map.rows * s + 0.5;
        std::ofstream csv;
        csv.open(ros::package::getPath("rob") + "/test" +
                 to_string(step_size) + ".csv");
        csv << step_size << endl;
        csv << comma_sep << s << endl;
        csv << endl;

        for (int i = 0; i < 100; i++)
        {
            RRT rrt(&map, start, step_size);
            float distance;
            int node_amount;
            rrt.connect(end, false, &node_amount, &distance);
            csv << node_amount << ";" << comma_sep << distance << std::endl;

            // ROS_INFO("%i", rrt.connect(end, false));
        }
        csv.close();
    }

    int step_size = map.rows * 0.03 + 0.5;
    std::ofstream csv;
    csv.open(ros::package::getPath("rob") + "/test_double.csv");
    csv << step_size << endl;
    csv << endl;

    for (int i = 0; i < 100; i++)
    {
        float distance;
        int node_amount;
        RRT rrt2(&map, start, step_size);
        rrt2.connect(end2, false, &node_amount, &distance);
        csv << node_amount << ";";

        RRT rrt(&map, start, step_size);
        rrt.connect(end, false, &node_amount, &distance);
        csv << node_amount << ";";
        rrt.connect(end2, false, &node_amount, &distance);
        csv << node_amount << endl;

        // ROS_INFO("%i", rrt.connect(end, false));
    }
    csv.close();

    cv::waitKey();
    /*while (true)
    {
        RRTPoint end = wait_mouse();
        ROS_INFO("%i", rrt.connect(end, false));

        cv::waitKey();
    }*/

    return 0;
}
