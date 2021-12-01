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


class Qlearn
{
	
private:

	//Siblings /children /connected states
	vector<State*> Qtable;

public:

Qlearn();
Qlearn();
~Qlearn();

};