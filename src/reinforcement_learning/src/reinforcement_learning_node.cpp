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

	// Loads in the map to be displayed------------------------------------------

	cv::Mat map;
	cv::namedWindow("Map", cv::WINDOW_NORMAL);
	string floorplan_path = ros::package::getPath("env_sim") + "/models/bigworld/meshes/floor_plan4.png";
	ROS_INFO("%s", floorplan_path.c_str());
	map = cv::imread(floorplan_path); // change this to change map

	//----------------Maps the states--------------------------------------------
	
	
	Envoriment big_world(map);
	Agent agent(&big_world);
	big_world.show_envoriment();
	big_world.get_state(1,1)->show_connected_states();
	agent.set_random_starting_state(map);
	//---------------------------------------------------------------------------

	cv::imshow("Map", map);
	while(true)
	{
		cv::imshow("Map", map);
		int key = cv::waitKey(0);
		switch (key)
		{
			case 'w':
			agent.take_action(map,Action::UP);
			break;
			case 's':
			agent.take_action(map,Action::DOWN);
			break;
			case 'a':
			agent.take_action(map,Action::LEFT);
			break;
			case 'd':
			agent.take_action(map,Action::RIGHT);
			break;
			case 'r':
			big_world.reset_map(map);
			break;
		}
	}

	return 0;
}