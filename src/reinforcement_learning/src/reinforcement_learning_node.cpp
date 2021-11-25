#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "state.h"
#include "agent.h"
#include "envoriment.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

	// Loads in the map to be displayed-----------

	cv::Mat map;
	cv::namedWindow("Map", cv::WINDOW_NORMAL);
	string floorplan_path = ros::package::getPath("env_sim") + "/models/bigworld/meshes/floor_plan2.png";
	ROS_INFO("%s", floorplan_path.c_str());
	map = cv::imread(floorplan_path); // change this to change map

	//----------------Maps the states--------------------------------------------
	
	Envoriment big_world(map);

	//---------------------------------------------------------------------------

	cv::imshow("Map", map);
	cv::waitKey(0);
	return 0;
}