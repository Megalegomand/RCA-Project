#include "rrt.h"

using namespace cv;
using namespace std;

RRT::RRT(Mat& map, RRTPoint start)
{
    init(map, start);
}

RRT::RRT(Mat& map)
{
    do 
    {
        RRTPoint start = random_point;
    } while (exists(start))
    // TODO Collision free instead

    init(map, start);
}

void RRT::init(cv::Mat& map, RRTPoint start)
{
    this->map = map;

    nodes = vector<RRTPoint>();
    nodes.push_back(start);
}

RRTPoint RRT::random_point()
{
    srand(clock());
    int x = rand() % map->rows;
    int y = rand() % map->cols;
    RRTPoint p(x, y);

    return p;
}

bool RRT::exists(RRTPoint& point)
{
    for (RRTPoint p : nodes)
    {
        if (p.get_x() == point->get_x() && p.get_y() == point->get_y())
        {
            return true;
        }
    }
    return false;
}

RRTPoint* RRT::closest(RRTPoint& p)
{
    if (nodes.size() == 0)
        return nullptr;

    float min_dist = p->dist(nodes[0]);
    RRTPoint* min_point = &nodes[0];
    for (int i = 1; i < nodes.size(); i++)
    {
        if (p->dist(nodes[i]) < min_dist)
        {
            min_dist = p->dist(nodes[i]);
            min_point = &nodes[i];
        }
    }

    return min_point;
}

bool RRT::build(RRTPoint end, bool vis = false)
{
    for (int i = 0; i < iterations; i++)
    {
        RRTPoint q = random_point();
        if (vis)
        {
            q.mark(map_vis, {255, 0, 255});
            imshow("Visual", map_vis);
            waitKey(0);
        }
    }
    return true;
}

RRT::~RRT()
{
}
