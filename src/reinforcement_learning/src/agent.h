#pragma once

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "state.h"
#include "envoriment.h"

using namespace std;
using namespace cv;

enum class Action
{
	UP,
	DOWN,
	LEFT,
	RIGHT
};

class Agent
{

private:
	// Envoriment Parameters
	Envoriment *envoriment;
	int n_states;
	pair<int, int> current_state;
	State *starting_state;

	// Agents position and color (current_State)
	vector<int> agent_color = {255, 0, 255}; // magenta color

public:
	Agent();
	Agent(Envoriment *map);
	State *get_current_state();
	void set_current_state(Mat map, int x, int y);
	void set_random_starting_state(Mat map);
	void take_action(Mat map, Action action);
	State *get_starting_state();
	State *get_agent_location();
	~Agent();
};