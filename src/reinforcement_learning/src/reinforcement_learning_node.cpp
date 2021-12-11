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
#include "Qlearn.h"
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
	Qlearn reinlearn(50000, &big_world, &agent);
	big_world.show_envoriment();
	big_world.get_state(1, 1)->show_connected_states();
	agent.set_random_starting_state(map);

	//---------------------------------------------------------------------------
	//cv::imshow("Map", map);

	//reinlearn.getAction();

	reinlearn.train(map);

	// map.at<Vec3b>(60, 18) ={208,224,64};
	// map.at<Vec3b>(32, 32) ={208,224,64};
	// map.at<Vec3b>(62, 54) ={208,224,64};
	// map.at<Vec3b>(0, 62) ={208,224,64};
	// map.at<Vec3b>(15, 8) ={208,224,64};
	// map.at<Vec3b>(77, 65) ={208,224,64};
	// map.at<Vec3b>(28, 52) ={208,224,64};
	// map.at<Vec3b>(62, 80) ={208,224,64};
	// map.at<Vec3b>(28, 37) ={208,224,64};
	// map.at<Vec3b>(32, 84) ={208,224,64};
	// map.at<Vec3b>(28, 52) ={208,224,64};
	// map.at<Vec3b>(0, 4) ={208,224,64};
	// map.at<Vec3b>(72, 65) ={208,224,64};
	// map.at<Vec3b>(28, 37) ={208,224,64};
	// map.at<Vec3b>(46, 66) ={208,224,64};
	// map.at<Vec3b>(20, 31) ={208,224,64};

	while (true)
	{
		cv::imshow("Map", map);
		cv::waitKey(0);
	}

	// while(true)
	// {
	// 	cv::imshow("Map", map);
	// 	int key = cv::waitKey(0);
	// 	switch (key)
	// 	{
	// 		case 'w':
	// 		agent.take_action(map,Action::UP);
	// 		break;
	// 		case 's':
	// 		agent.take_action(map,Action::DOWN);
	// 		break;
	// 		case 'a':
	// 		agent.take_action(map,Action::LEFT);
	// 		break;
	// 		case 'd':
	// 		agent.take_action(map,Action::RIGHT);
	// 		break;
	// 		case 'r':
	// 		big_world.reset_map(map);
	// 		break;
	// 	}
	// }

	return 0;
}