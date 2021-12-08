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
	Vec3b color_val;

	//Siblings /children /connected states
	vector<State *> connected_states;

	//state status (current or not)(visted or not)
	bool current = false;
	bool isVisted = false;
	int VisitedCounter = 0;

	//state reward of current state
	int reward;

	// Q properties (UP, LOW, LEFT, RIGHT)
	vector<double> QValues = {0, 0, 0, 0};

	// points properties
	int thickness = -1;

public:
	State();
	State(int x_coordinate_, int y_coordinate_, int reward_); // Mat map // vector <State*> connections );
	pair<int, int> get_location();
	void set_color_val(int b, int g, int r);
	Vec3b get_color_val();
	void set_current_state(Mat map, bool status);
	void set_reward(int value);
	bool get_isVisted();
	void set_isVisted();
	int set_VisitedCounter();
	int reset_VisitedCounter();
	State *best_choice();
	int get_VisitedCounter();
	void set_QValues(int index, double q_val);
	vector<double> get_QValues();
	int get_reward();
	void set_connected_states(State *child);
	void show_connected_states();
	vector<State *> get_connected_states();
	~State();
};
