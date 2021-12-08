#include "particle.h"

using namespace cv;
using namespace sensor_msgs;
using namespace std;

Particle::Particle(float x, float y, float angle, Mat *map)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
    this->map = map;
}

float Particle::wall_distance(float angle)
{
    cv::Rect rect(cv::Point(), map->size());

    float dist = 0.0f;
    while (true)
    {
        Point p((int)x + cos(this->angle + angle) * dist + 0.5f,
                (int)y + sin(this->angle + angle) * dist + 0.5f);
        if (map->at<Vec3b>(p) == Vec3b(0, 0, 0) || !rect.contains(p))
        {
            break;
        }

        dist += 0.1f;
    }
    return dist;
}

void Particle::mark(Mat *vis_map)
{
    // Round to nearest int
    Point p1((int)x + 0.5f, (int)y + 0.5f);
    Point p2((int)x + cos(angle) * MARK_DISTANCE + 0.5f,
             (int)y + sin(angle) * MARK_DISTANCE + 0.5f);
    line(*vis_map, p1, p2, Vec3b(255, 0, 0));
}

float Particle::sensor_update(const LaserScanConstPtr &scan)
{
    double likelihood = 1.0f;
    for (int i = 0; i < scan->ranges.size(); i++)
    {
        float angle = scan->angle_min + scan->angle_increment * i;
        float measurement = scan->ranges[i];
        float estimation = wall_distance(angle);
        if (scan->ranges[i] > scan->range_max ||
            scan->ranges[i] < scan->range_min)
        {
            continue;
        }
        ROS_INFO("%f", exp(-0.5f * pow((estimation - measurement), 2.0f) / pow(LASER_STD, 2.0f)) /
            (LASER_STD * sqrt(2 * M_PI)));
        likelihood *=
            exp(-pow((estimation - measurement), 2) / pow(LASER_STD, 2) / 2.0) /
            (LASER_STD * sqrt(2 * M_PI));
    }
    
    ROS_INFO("likelihood: %f", likelihood);
    return 0.0f;
}

Particle::~Particle()
{
}
