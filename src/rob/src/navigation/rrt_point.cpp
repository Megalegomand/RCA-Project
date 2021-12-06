#include "rrt_point.h"

using namespace cv;

RRTPoint::RRTPoint(int x, int y)
{
    this->x = x;
    this->y = y;
}

int RRTPoint::get_x()
{
    return x;
}

int RRTPoint::get_y()
{
    return y;
}

float RRTPoint::dist(RRTPoint& p)
{
    return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
}

void RRTPoint::mark(Mat* map, Vec3b color)
{
    map->at<Vec3b>(y, x) = color;
}

RRTPoint::~RRTPoint()
{
}
