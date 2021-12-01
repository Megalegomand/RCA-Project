#pragma once

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

class Agent
{
	
private:
	// Envoriment Parameters
	int n_states;
	State* current_state;

	// Q-learning parameters
	float lr = 0.001;
    float gamma = 0.99;
    float exploration_proba = 1.0;
    float exploration_proba_decay = 0.005;
    float batch_size = 32;

	// Memory / buffer
	vector<State*> memory;
	int memory_size = 2000;

	// Agents position and color (current_State)
	


public:
    Agent();
    Agent(Envoriment *map);
	void take_action();
	float set_exploration_proba();
	float get_lr();
	float get_gamma();
	float get_exploration_proba();
	float get_exploration_proba_decay();
	float get_batch_size();
	void store_episode();
	void train();
    ~Agent();
};