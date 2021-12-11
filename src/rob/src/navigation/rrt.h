#pragma once

#include "assert.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rrt_point.h"
#include "vector"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class RRT
{
  private:
    cv::Mat *map;
    cv::Mat map_vis;
    std::vector<RRTPoint> nodes;
    int iterations = 10000;
    int step_size = 10;
    float step_col_div = 0.1f;

    RRTPoint random_point();
    bool exists(RRTPoint *point);
    int closest(RRTPoint *p);

    void init(cv::Mat *map, RRTPoint start, int step_size_ = 10);

  public:
    RRT(cv::Mat *map, RRTPoint start, int step_size_ = 10);
    bool connect(RRTPoint end, bool vis, int *node_amount, float *distance);
    void visualize();
    RRTPoint wait_mouse();
    void mouse_callback(int event, int x, int y, int flags, void *userdata);

    ~RRT();
};
