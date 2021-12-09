#include "particle.h"

using namespace cv;
using namespace sensor_msgs;
using namespace std;

Particle::Particle(float x, float y, float angle, Mat *map)
{
    map2real = REAL_WIDTH / map->cols;

    this->x = x;
    this->y = y;
    this->angle = angle;
    this->map = map;
}

float Particle::wall_distance(float angle)
{
    float x_diff = cos(this->angle + angle) + 0.5f;
    float y_diff = sin(this->angle + angle) + 0.5f;
    float x_ = x;
    float y_ = y;
    while (true)
    {
        int x_int = x_;
        int y_int = y_;
        if (map->at<Vec3b>(y_int, x_int) == Vec3b(0, 0, 0))
        {
            break;
        }
        x_ += x_diff;
        y_ += y_diff;
    }
    return sqrt(pow(x - round(x_), 2) + pow(y - round(y_), 2)) * map2real;
}

void Particle::mark(Mat *vis_map)
{
    // Round to nearest int
    Point p1((int)x + 0.5f, (int)y + 0.5f);
    Point p2((int)x + cos(angle) * MARK_DISTANCE * (weight) * 1000 + 0.5f,
             (int)y + sin(angle) * MARK_DISTANCE * (weight) * 1000 + 0.5f);
    line(*vis_map, p1, p2, Vec3b(255, 0, 0), 1);
}

double Particle::get_likelihood(const LaserScanConstPtr &scan)
{
    double likelihood = 1.0f;
    for (int d = 0; d < LASER_DIVISIONS; d++)
    {
        int i = d * scan->ranges.size() / LASER_DIVISIONS;
        float angle = scan->angle_min + scan->angle_increment * i;
        float measurement = scan->ranges[i];
        float estimation = wall_distance(angle);
        if (scan->ranges[i] > scan->range_max ||
            scan->ranges[i] < scan->range_min)
        {
            continue;
        }
        likelihood *= exp(-pow((estimation - measurement), 2) /
                          pow(LASER_STD, 2) / 2.0) /
                      (LASER_STD * sqrt(2 * M_PI));
    }
    weight = likelihood;
    return likelihood;
}

Particle::~Particle()
{
}
