#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "State.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

	
	// Loads in the map to be displayed
	cv::Mat map;
	cv::namedWindow("Map", cv::WINDOW_NORMAL);
	map = cv::imread("C:/Users/rasm4/Downloads/floor_plan.png"); // change this to change map
	

	State one(&map, 8, 9);
	State two(&map, 23, 9);
	State tree(&map, 23, 28);
	State four(&map, 35, 38);
	State five(&map, 42, 24);
	State six(&map, 68, 24);
	State seven(&map, 42, 10);
	State eight(&map, 68, 10);
	State nine(&map, 92, 8);
	State ten(&map, 115, 20);
	State eleven(&map, 35, 60);
	State tvelve(&map, 101, 40);
	State thirteen(&map, 114, 62);
	State fourteen(&map, 68, 55);
	State fiveteen(&map, 78, 70);
	State sixteen(&map, 55, 75);
	State seventeen(&map, 8, 45);

	ten.setCurrentState(&map, true);
	ten.setCurrentState(&map, false);
	ten.setCurrentState(&map, true);

cv::imshow("Map", map);


	cv::waitKey(0);
	return 0;
}