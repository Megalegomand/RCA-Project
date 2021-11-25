#include "state.h"

State::State()
{
}
//vector child / sibling vector af s
// State::State(Mat map, int x_coordinate_, int y_coordinate_, int reward_) // vector <State*> connections)
// {
// 	x_coordinate = x_coordinate_;
// 	y_coordinate = y_coordinate_;
	
// 	reward_ = reward;

// 	center_pt = Point(y_coordinate, x_coordinate + 2);

// 	p1 = Point(x_coordinate, y_coordinate);
// 	p2 = Point(x_coordinate + 2, y_coordinate + 2);
	
// 	// Draw a dot on the map to represent the states location
// 	rectangle(map, p1, p2,
// 		Scalar(0, 255, 0),
// 		thickness, LINE_8);
// }
State::State( int x_coordinate_, int y_coordinate_, int reward_) // vector <State*> connections)
{
	x_coordinate = x_coordinate_;
	y_coordinate = y_coordinate_;
	
	reward_ = reward;

	center_pt = Point(y_coordinate, x_coordinate + 2);

	p1 = Point(x_coordinate, y_coordinate);
	p2 = Point(x_coordinate + 2, y_coordinate + 2);
	
}

void State::set_current_state(Mat map, bool status) 
{
	if (status == true)
	{
		cv::Point xy;
		xy.x = x_coordinate + 1; // the +1 makes sure it is placed in the middle of the square
		xy.y = y_coordinate + 1; // the +1 makes sure it is placed in the middle of the square
		map.at<Vec3b>(xy) = { 0,0,255 };
		current = true;
	}
	else if (status == false)
	{
		cv::Point xy;
		xy.x = x_coordinate + 1; // the +1 makes sure it is placed in the middle of the square
		xy.y = y_coordinate + 1; // the +1 makes sure it is placed in the middle of the square
		map.at<Vec3b>(xy) = { 0,255,0 };
		current = false;
	}
	

}
void State::set_reward(int value)
{
	reward = value;
}

void State::set_connected_states(Mat map, vector<State*>child)
{
	connected_states = child;

	// Draws a line between connected states     line(img, pt1, pt2, color, thickness, lineType, shift)
	for(int i = 0; i < connected_states.size(); i++)
	{
		line(map, center_pt,connected_states[i]->get_center(),Scalar(255, 0, 0) ,1, LINE_8, 0);
	}
}

int State::get_reward()
{
	return reward;
}

Point State::get_center()
{
	return center_pt;
}

// State::get_current_state()
// {
// 	return State;
// }

State::~State()
{
}
