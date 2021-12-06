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

float RRTPoint::dist(RRTPoint *p)
{
    return sqrt((x - p->x) * (x - p->x) + (y - p->y) * (y - p->y));
}

float RRTPoint::angle(RRTPoint *p)
{
    return atan2(p->y - y, p->x - x);
}

void RRTPoint::mark(Mat *map, Vec3b color)
{
    map->at<Vec3b>(y, x) = color;
}

bool RRTPoint::collision_line(cv::Mat *map, RRTPoint *p)
{
    float a = angle(p);
    for (int i = 0; i < ceil(dist(p)); i++)
    {
        float lx = cos(x) * i + x;
        float ly = sin(x) * i + y;
        if (map->at<Vec3b>((int)floor(lx), (int)floor(ly)) == Vec3b(0, 0, 0) ||
            map->at<Vec3b>((int)ceil(lx), (int)floor(ly)) == Vec3b(0, 0, 0) ||
            map->at<Vec3b>((int)floor(lx), (int)ceil(ly)) == Vec3b(0, 0, 0) ||
            map->at<Vec3b>((int)ceil(lx), (int)ceil(ly)) == Vec3b(0, 0, 0))
        {
            return true;
        }
    }
    return false;
}

RRTPoint::~RRTPoint()
{
}
