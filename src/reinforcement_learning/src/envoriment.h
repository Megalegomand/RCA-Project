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
        vector<vector<State>>* get_envoriment(); 
        void show_enoriment();
        ~Envoriment();

};