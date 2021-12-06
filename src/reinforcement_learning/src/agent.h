#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "state.h"
#include "envoriment.h"
#include "Qlearn.h"

using namespace std;
using namespace cv;

enum class Action { UP, DOWN, LEFT, RIGHT };

class Agent
{
	
private:
	// Envoriment Parameters
	Envoriment* envoriment;
	int n_states;
	State* current_state;
	State* starting_state;

	// learning parameters
	float lr = 0.001;
    float gamma = 0.99;
    float exploration_proba = 1.0;
    float exploration_proba_decay = 0.005;
    float batch_size = 32;

	// Memory / buffer
	vector<State*> memory;
	int memory_size = 2000;

	// Possible actions
	

	// Agents position and color (current_State)
	vector <int> agent_color ={255,0,255}; // magenta color


public:

    Agent();
    Agent(Envoriment *map);
	void set_current_state(Mat map, int x, int y);
	void set_random_starting_state(Mat map);
	void take_action(Mat map, Action action); // i Q learn
	float set_exploration_proba();
	float get_lr();
	float get_gamma();
	float get_exploration_proba();
	float get_exploration_proba_decay();
	float get_batch_size();
	State* get_agent_location();
	void store_episode();
	/*void train(Qlearn* algo, int number_of_episode, int interations_pr_epi);*/
    ~Agent();

};