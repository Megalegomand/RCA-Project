#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rrt_point.h"
#include "vector"
#include "ros/ros.h"
#include "assert.h"

class RRT
{
private:
    cv::Mat* map;
    cv::Mat map_vis;
    std::vector<RRTPoint> nodes;
    int iterations = 5000;
    int step_size = 10;

    RRTPoint random_point();
    bool exists(RRTPoint* point);
    int closest(RRTPoint* p);

    void init(cv::Mat* map, RRTPoint start);
public:
    RRT(cv::Mat* map, RRTPoint start);
    bool build(RRTPoint end, bool vis = false);
    void visualize();
    RRTPoint wait_mouse();
    void mouse_callback(int event, int x, int y, int flags, void*userdata);
    
    ~RRT();
};
