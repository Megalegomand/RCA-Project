#include "rrt.h"

using namespace cv;
using namespace std;

RRT::RRT(Mat *map, RRTPoint start)
{
    init(map, start);
}

void RRT::init(cv::Mat *map, RRTPoint start)
{
    this->map = map;

    map_vis = map->clone();

    nodes = vector<RRTPoint>();
    nodes.push_back(start);

    mouse_clicked.acquire();
}

RRTPoint RRT::random_point()
{
    srand(clock());

    int x = rand() % map->cols;
    int y = rand() % map->rows;

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

int RRT::closest(RRTPoint *p)
{
    if (nodes.size() == 0)
        return -1;

    int min_index = 0;
    float min_dist = p->dist(&nodes[min_index]);

    for (int i = 1; i < nodes.size(); i++)
    {
        if (p->dist(&nodes[i]) < min_dist)
        {
            min_index = i;
            min_dist = p->dist(&nodes[i]);
        }
    }

    return min_index;
}

bool RRT::build(RRTPoint end, bool vis)
{
    for (int i = 0; i < iterations; i++)
    {
        RRTPoint q = random_point();
        int q_near_index = closest(&q);
        RRTPoint *q_near = &nodes[q_near_index];
        assert(q_near != nullptr);

        float angle = q_near->angle(&q);
        int step = std::min(step_size, (int)q.dist(q_near));

        RRTPoint q_new = RRTPoint(q_near->get_x() + cos(angle) * step,
                                  q_near->get_y() + sin(angle) * step);
        q_new.set_parent(q_near_index);

        /*ROS_INFO("Q: (%i, %i), Q_near: (%i, %i), Q_new: (%i, %i)", q.get_x(),
                 q.get_y(), q_near->get_x(), q_near->get_y(), q_new.get_x(),
                 q_new.get_y());*/

        if (q_near->collision_line(map, &q_new) || exists(&q_new) ||
            q_near->collision(map) || q_new.collision(map))
        {
            //ROS_INFO("Collision");
            continue;
        }

        if (vis)
        {
            visualize();

            q_near->mark(&map_vis, Vec3b(0, 0, 255));
            q.mark(&map_vis, Vec3b(255, 0, 255));
            q_new.mark(&map_vis, Vec3b(255, 0, 0));
        }

        nodes.push_back(q_new);
    }

    return true;
}

void RRT::visualize()
{
    map_vis = map->clone();
    namedWindow("Visual", WINDOW_KEEPRATIO);

    for (RRTPoint p : nodes)
    {
        p.mark(&map_vis, Vec3b(0, 255, 0));
        int parent = p.get_parent();
        if (p.get_parent() != -1)
        {
            Point p1(p.get_x(), p.get_y());
            Point p2(nodes[parent].get_x(), nodes[parent].get_y());
            line(map_vis, p1, p2, Vec3b(128, 128, 128), 1);
        }
    }
    imshow("Visual", map_vis);
    // waitKey(0);
}

RRT::~RRT()
{
}
