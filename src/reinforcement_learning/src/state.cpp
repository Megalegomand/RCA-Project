#include "state.h"

State::State()
{
}

State::State(int x_coordinate_, int y_coordinate_, int reward_) // vector <State*> connections)
{
	x_coordinate = x_coordinate_;
	y_coordinate = y_coordinate_;
	
	reward = reward_;

}

pair<int, int> State::get_location()
{	
	pair <int, int> location = { x_coordinate, y_coordinate };
	return location;
}
void State::set_color_val(int b, int g, int r)
{
	color_val[0] = b;
	color_val[1] = g;
	color_val[2] = r;
}
Vec3b State::get_color_val()
{
	return color_val;
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
		isVisted = true;
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
bool State::get_isVisted()
{
	return isVisted;
}
bool State::set_isVisted()
{
	return isVisted = true;
}
int State::set_VisitedCounter()
{
	return VisitedCounter = VisitedCounter+1;
}
int State::reset_VisitedCounter()
{
	return VisitedCounter = 0;
}
int State::get_VisitedCounter()
{
	return VisitedCounter;
}
void State::set_QValues(int index, double q_val)
{
	QValues[index] = q_val;
}
vector<double> State::get_QValues()
{
	return QValues;
}
int State::get_reward()
{
	return reward;
}

void State::set_connected_states(State* child)
{
	return connected_states.push_back(child);
}
vector<State*> State::get_connected_states()
{
	return connected_states;
}

void State::show_connected_states()
{
	for(int i = 0; i < connected_states.size(); i++)
	{
		cout << "X =" << connected_states[i]->get_location().first << "," << "Y =" << connected_states[i]->get_location().second << endl;
	}
}

State::~State()
{
}
