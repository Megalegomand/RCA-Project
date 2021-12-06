#include "rrt.h"

using namespace cv;
using namespace std;

RRT::RRT(Mat *map, RRTPoint start)
{
    init(map, start);
}

RRT::RRT(Mat *map)
{
    RRTPoint start = random_point();
    while (exists(&start))
    {
        RRTPoint start = random_point();
    }
    // TODO Collision free instead

    init(map, start);
}

void RRT::init(cv::Mat *map, RRTPoint start)
{
    this->map = map;

    map_vis = map->clone();

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

bool RRT::exists(RRTPoint *point)
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

RRTPoint *RRT::closest(RRTPoint *p)
{
    if (nodes.size() == 0)
        return nullptr;

    float min_dist = p->dist(&nodes[0]);
    RRTPoint *min_point = &nodes[0];
    for (int i = 1; i < nodes.size(); i++)
    {
        if (p->dist(&nodes[i]) < min_dist)
        {
            min_dist = p->dist(&nodes[i]);
            min_point = &nodes[i];
        }
    }

    return min_point;
}

bool RRT::build(RRTPoint end, bool vis)
{
    for (int i = 0; i < iterations; i++)
    {
        RRTPoint q = random_point();
        RRTPoint *q_near = closest(&q);

        float angle = q_near->angle(&q);
        int step = std::min(step_size, (int)q.dist(q_near));

        RRTPoint q_new = RRTPoint(q_near->get_x() + cos(angle) * step,
                                  q_near->get_y() + sin(angle) * step);

        ROS_INFO("Q: (%i, %i), Q_near: (%i, %i), Q_new: (%i, %i)", q.get_x(),
                 q.get_y(), q_near->get_x(), q_near->get_y(), q_new.get_x(),
                 q_new.get_y());

        if (q_near->collision_line(map, &q_new) || exists(&q_new))
        {
            ROS_INFO("Collision");
            continue;
        }

        ROS_INFO("Kage");
        nodes.push_back(q_new);
        ROS_INFO("Kage");

        if (vis)
        {
            map_vis = map->clone();
            namedWindow("Visual", WINDOW_KEEPRATIO);
            
            for (RRTPoint p : nodes)
            {
                p.mark(&map_vis, Vec3b(255, 0, 255));
            }
            q_near->mark(&map_vis, Vec3b(0, 0, 255));
            q_new.mark(&map_vis, Vec3b(255, 0, 0));
            imshow("Visual", map_vis);
            waitKey(0);
        }
    }
    return true;
}

RRT::~RRT()
{
}
