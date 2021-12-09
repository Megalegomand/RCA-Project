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
	//ROS_INFO("x: %i, y: %i", x_coordinate, y_coordinate);
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
void State::set_isVisted()
{
	isVisted = true;
}
void State::set_VisitedCounter()
{
	VisitedCounter++;
}
void State::reset_VisitedCounter()
{
	VisitedCounter = 0;
}
State* State::best_choice()
{
	double best = -1000.0;
	int state_index;
	for (int i = 0; i < get_QValues().size(); i++)
	{
		if(best < get_QValues()[i] && get_connected_states()[i]->get_color_val() != Vec3b(0,0,0))
		{
			best = get_QValues()[i];
			state_index = i;
		}
	}
	return get_connected_states()[state_index];
}
int State::get_VisitedCounter()
{
	return VisitedCounter;
}
void State::set_QValues(int index, double q_val)
{
	while(get_connected_states()[index]->get_color_val() == Vec3b(0,0,0) )
	{
		index++;
		index = index % get_connected_states().size();
	}
	QValues[index] = q_val;
}
vector<double> State::get_QValues()
{
	return QValues;
}
vector<double> State::get_valid_Qval()
{
	Valid_Qval.clear();
	for (int i = 0; i < QValues.size(); i++)
	{
		if(get_connected_states()[i]->get_color_val() != Vec3b(0,0,0))
		{
			Valid_Qval.push_back(QValues[i]);
		}
	}
	return Valid_Qval;	
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
