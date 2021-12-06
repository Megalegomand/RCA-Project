#pragma once

#include <iostream>
#include <math.h>
#include <random>
#include <algorithm>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "state.h"
#include "agent.h"
#include "envoriment.h"

using namespace std;
using namespace cv;


class Qlearn
{
	
private:

	//States
	Envoriment* states;
	Agent* robot;
	//Q-table
	vector<vector<State*>> Qtable;
	//Number of episodes
	int n_episodes;
	// Parameters
	float lr = 0.01;
    float gamma = 0.3;
    float epsilon = 1.0;
    float epsilon_decay = 0.005;
	float min_exploration_rate = 0.01;
	float max_exploration_rate = 1.0;
	int maxSteps = 100;


public:

Qlearn(); // ha' en take action()
Qlearn(int n_episodes_, Envoriment* states_, Agent* agent_);
void doAction();
State* getAction();
void doEpisode();
void train();
void displayQTable();
void QUpdate();
void implementAgent();
void ExportData();
~Qlearn();

};