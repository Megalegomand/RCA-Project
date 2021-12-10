#pragma once
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "state.h"

using namespace std;
using namespace cv;

class Envoriment
{
private:
    vector<vector<State>> envoriment;

public:
    Envoriment();
    Envoriment(Mat map);
    vector<vector<State>> *get_envoriment();
    State *get_state(int x, int y);
    void reset_map(Mat map); // does not work
    void show_envoriment();
    ~Envoriment();
};