#pragma once

#include <iostream>
#include <math.h>
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

	vector<vector<State*>> Qtable;

public:

Qlearn(); // ha' en take action()
Qlearn(Envoriment* states, Agent* agent);
void QBellmanEq();
void QUpdate();
~Qlearn();

};