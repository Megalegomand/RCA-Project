#pragma once

#include "vector"
#include "math.h"
#include <opencv2/core/core.hpp>

class RRTPoint
{
private:
    int x, y;
    RRTPoint* parent;
public:
    RRTPoint(int x, int y);
    int get_x();
    int get_y();
    float dist(RRTPoint* p);
    float angle(RRTPoint* p);
    void mark(cv::Mat* map, cv::Vec3b color);
    bool collision_line(cv::Mat* map, RRTPoint *p);
    bool collision(cv::Mat* map);
    void set_parent(RRTPoint* p);
    ~RRTPoint();
};
