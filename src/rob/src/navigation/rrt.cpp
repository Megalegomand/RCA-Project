#include "rrt.h"

using namespace cv;

RRT::RRT(Mat* map, RRTPoint start)
{
    this->map = map;
}

RRT::~RRT()
{
}
