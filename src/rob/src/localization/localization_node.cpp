#include "mcl.h"
#include "ros/package.h"
#include "ros/ros.h"
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

    cv::resize(map, map, cv::Size(), 4.0f, 4.0f, cv::INTER_NEAREST);

    ROS_INFO("Map size: (%i, %i)", map.cols, map.rows);

    MCL mcl(&map);
    mcl.visualize();
    mcl.randomize_particles();

    ros::spin();
    /*
    mcl.randomize_particles();
    mcl.visualize();

    waitKey();
*/
    return 0;
}
