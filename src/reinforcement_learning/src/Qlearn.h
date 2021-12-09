#pragma once

#include "agent.h"
#include <iostream>
#include <math.h>
#include <random>
#include <algorithm>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "state.h"
#include "envoriment.h"
#include "fstream"

class Qlearn
{

private:
	//States
	Envoriment *states;
	Agent *robot;
	//Q-table
	vector<vector<State *>> Qtable;
	int index_action;
	//Number of episodes
	int n_episodes;
	// Parameters
	double lr = 0.01;
	double gamma = 0.3;
	double epsilon = 1.0;
	double epsilon_decay = 0.005;
	double min_exploration_rate = 0.01;
	double max_exploration_rate = 1.0;
	int maxSteps = 10;

	// Results
	double maxReward;

	vector<double>expectedPrEpisode;
	vector<int>AllEpisodes;
	vector<double>AllEpsilon;
	vector<double>All_lr;

	ofstream DataCollection;
	string filename = "QlearnTestNum1";

public:
	Qlearn();
	Qlearn(int n_episodes_, Envoriment *states_, Agent *agent_);
	State* getAction();
	State* doAction(Mat map);
	int get_largestIndex(int ele, vector<State *> set_of_valid_actions);
	void doEpisode(Mat map);
	void train(Mat map);
	void displayQTable();
	void implementAgent(Mat map);
	void ExportData();
	~Qlearn();
};