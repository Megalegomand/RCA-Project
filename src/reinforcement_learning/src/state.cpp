#include "state.h"

State::State()
{
}

State::State( int x_coordinate_, int y_coordinate_, int reward_) // vector <State*> connections)
{
	x_coordinate = x_coordinate_;
	y_coordinate = y_coordinate_;
	
	reward = reward_;

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

void State::set_connected_states(State* child)
{
	return connected_states.push_back(child);
}
vector<State*> State::get_connected_states()
{
	return connected_states;
}

int State::get_reward()
{
	return reward;
}

State::~State()
{
}
