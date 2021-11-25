#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "state.h"
#include "agent.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <string>

using namespace std;
using namespace cv;

// Mapping of states in envoriment
// White pix is neutral:
// Black pix is walls:
// Red   pix is traps:
// Green pix is low reward:
// Blue  pix is high reward:

void mapping(cv::Mat map)
{
	int rows = map.rows;
	int cols = map.cols;

	int white_val = 0;
	int black_val = -1;
	int red_val = -5;
	int green_val = 1;
	int blue_val = 5;

	vector<vector<State>> envoriment;

	for (int i = 0; i < rows; i++)
	{
		vector<State> state_temp;

		for (int y = 0; y < cols; y++)
		{
			vector<int> pixel = {(int)map.at<cv::Vec3b>(i, y)[0], (int)map.at<cv::Vec3b>(i, y)[1], (int)map.at<cv::Vec3b>(i, y)[2]};

			// if pixel is white
			if (pixel[0] == 255 & pixel[1] == 255 & pixel[2] == 255)
			{

				State white_state(i, y, white_val);
				state_temp.push_back(white_state);

			}
			// if pixel is black
			if (pixel[0] == 0 & pixel[1] == 0 & pixel[2] == 0)
			{;

				State black_state(i, y, black_val);
				state_temp.push_back(black_state);

			}
			// if pixel is red
			if (pixel[0] == 0 & pixel[1] == 0 & pixel[2] == 255)
			{

				State red_state(i, y, red_val);
				state_temp.push_back(red_state);

			}
			// if pixel is green
			if (pixel[0] == 0 & pixel[1] == 255 & pixel[2] == 0)
			{

				State green_state(i, y, green_val);
				state_temp.push_back(green_state);

			}
			// if pixel is blue
			if (pixel[0] == 255 & pixel[1] == 0 & pixel[2] == 0)
			{

				State blue_state(i, y, blue_val);
				state_temp.push_back(blue_state);

			}
		}
		envoriment.push_back(state_temp);
		state_temp.clear();
	}

	for (auto i : envoriment)
	{
		for (auto y : i)
		{
			cout << y.get_reward();
		}
		cout << endl;
	}
}

int main(int argc, char **argv)
{

	// Loads in the map to be displayed

	cv::Mat map;
	cv::namedWindow("Map", cv::WINDOW_NORMAL);
	string floorplan_path = ros::package::getPath("env_sim") + "/models/bigworld/meshes/floor_plan2.png";
	ROS_INFO("%s", floorplan_path.c_str());
	map = cv::imread(floorplan_path); // change this to change map

	// State one(map, 8, 9, 1);
	// State two(map, 23, 9, 1);
	// State three(map, 23, 28, 1);
	// State four(map, 35, 38, 1);
	// State five(map, 42, 24, 1);
	// State six(map, 68, 24, 1);
	// State seven(map, 42, 10, 1);
	// State eight(map, 68, 10, 1);
	// State nine(map, 92, 8, 1);
	// State ten(map, 115, 20, 1);
	// State eleven(map, 35, 60, 1);
	// State tvelve(map, 101, 40, 1);
	// State thirteen(map, 114, 62, 1);
	// State fourteen(map, 68, 55, 1);
	// State fiveteen(map, 78, 70, 1);
	// State sixteen(map, 55, 75, 1);
	// State seventeen(map, 8, 45, 1);

	// vector<State *> connect_one = {&ten};
	// vector<State *> connect_two = {&one, &two};
	// vector<State *> connect_three = {&one, &two};
	// vector<State *> connect_four = {&one, &two};
	// vector<State *> connect_five = {&one, &two};
	// vector<State *> connect_six = {&one, &two};
	// vector<State *> connect_seven = {&one, &two};
	// vector<State *> connect_eight = {&one, &two};
	// vector<State *> connect_nine = {&one, &two};
	// vector<State *> connect_ten = {&one, &two};
	// vector<State *> connect_eleven = {&one, &two};
	// vector<State *> connect_tvelve = {&one, &two};
	// vector<State *> connect_thirteen = {&one, &two};
	// vector<State *> connect_fourteen = {&one, &two};
	// vector<State *> connect_fiveteen = {&one, &two};
	// vector<State *> connect_sixteen = {&one, &two};
	// vector<State *> connect_seventeen = {&one, &two};

	// one.set_connected_states(map, connect_one);
	// two.set_connected_states(map, connect_two);
	// three.set_connected_states(map, connect_three);
	// four.set_connected_states(map, connect_four);
	// five.set_connected_states(map, connect_five);
	// six.set_connected_states(map, connect_six);
	// seven.set_connected_states(map, connect_seven);
	// eight.set_connected_states(map, connect_eight);
	// nine.set_connected_states(map, connect_nine);
	// ten.set_connected_states(map, connect_ten);
	// eleven.set_connected_states(map, connect_eleven);
	// tvelve.set_connected_states(map, connect_tvelve);
	// thirteen.set_connected_states(map, connect_thirteen);
	// fourteen.set_connected_states(map, connect_fourteen);
	// fiveteen.set_connected_states(map, connect_fiveteen);
	// sixteen.set_connected_states(map, connect_sixteen);
	// seventeen.set_connected_states(map, connect_seventeen);

	// ten.set_current_state(map, true);
	// ten.set_current_state(map, false);
	// ten.set_current_state(map, true);

	// cv::line(map, one.get_center(), ten.get_center(),cv::Scalar(0,255,0), 1, cv::LINE_8, 0);
	// cv::line(map, Point(8,9), ten.get_center(),cv::Scalar(255,0,0), 1, cv::LINE_8, 0);
	// cv::line(map, one.get_center(), Point(115,20),cv::Scalar(255,0,0), 1, cv::LINE_8, 0);

	cv::imshow("Map", map);
	mapping(map);

	cv::waitKey(0);
	return 0;
}