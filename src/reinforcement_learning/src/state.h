#pragma once
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"

using namespace std;
using namespace cv;

class State
{
	
private:
	// location of State in the envoriment
	int x_coordinate;
	int y_coordinate;

	//Siblings /children /connected states
	vector<State*> connected_states;

	//state status (current or not)
	bool current = false;

	//state reward of current state
	int reward;

	// points properties
	int thickness = -1;

public:

State();
State(int x_coordinate_, int y_coordinate_, int reward_); // Mat map // vector <State*> connections );
pair get_location();
void set_current_state(Mat map, bool status);
void set_reward(int value);
int get_reward();
void set_connected_states(State* child);
void show_connected_states();
vector<State*> get_connected_states();

~State();

};

