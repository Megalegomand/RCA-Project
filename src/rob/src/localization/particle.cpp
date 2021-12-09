#include "particle.h"

using namespace cv;
using namespace sensor_msgs;
using namespace std;

Particle::Particle(float x, float y, float angle, Mat *map, MCL *parent)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
    this->map = map;
    this->parent = parent;
}

float Particle::wall_distance(float angle)
{
    Mat vis_map = map->clone();
    mark(&vis_map);

    ROS_INFO("a1 %f", this->angle + angle);
    float x_diff = cos(this->angle + angle) + 0.5f;
    float y_diff = sin(this->angle + angle) + 0.5f;
    float x_ = parent->real2map_x(x);
    float y_ = parent->real2map_y(y);
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
    float distance = sqrt(pow(x - parent->map2real_x(x_), 2) +
                          pow(y - parent->map2real_y(y_), 2));

    ROS_INFO("a2 %f", this->angle + angle);
    Point p2(parent->real2map_x(x + cos(this->angle + angle) * distance),
             parent->real2map_y(y + sin(this->angle + angle) * distance));
    vis_map.at<Vec3b>(p2) = Vec3b(0, 255, 0);
    Point p3(x_, y_);
    vis_map.at<Vec3b>(p3) = Vec3b(0, 0, 255);
    Point p4(parent->real2map_x(x + cos(this->angle + angle) * 10),
             parent->real2map_y(y + cos(this->angle + angle) * 10));
    vis_map.at<Vec3b>(p4) = Vec3b(255, 0, 0);
    Point p5(parent->real2map_x(x + cos(this->angle + angle) * 5),
             parent->real2map_y(y + cos(this->angle + angle) * 5));
    vis_map.at<Vec3b>(p5) = Vec3b(255, 0, 0);

    imshow("Vis", vis_map);

    return sqrt(pow(x - parent->map2real_x(x_), 2) +
                pow(y - parent->map2real_y(y_), 2));
}

void Particle::mark(Mat *vis_map)
{
    // Round to nearest int
    Point p1(parent->real2map_x(x), parent->real2map_y(y));
    Point p2(p1.x + cos(angle) * MARK_DISTANCE,
             p1.y + sin(angle) * MARK_DISTANCE);
    line(*vis_map, p1, p2, Vec3b(255, 0, 0), 1);
    vis_map->at<Vec3b>(p1) = Vec3b(0, 0, 255);
}

double Particle::get_likelihood(const LaserScanConstPtr &scan)
{
    Mat vis_map = map->clone();
    mark(&vis_map);

    double likelihood = 1.0f;
    for (int d = 0; d < LASER_DIVISIONS; d++)
    {
        ROS_INFO("min: %f, max: %f", scan->angle_min, scan->angle_max);
        int i = d * scan->ranges.size() / LASER_DIVISIONS;
        float meas_angle = scan->angle_min + scan->angle_increment * i;
        float measurement = scan->ranges[i];
        float estimation = wall_distance(meas_angle);

        Point p2(parent->real2map_x(x + cos(meas_angle + angle) * estimation),
                 parent->real2map_y(y + sin(meas_angle + angle) * estimation));
        vis_map.at<Vec3b>(p2) = Vec3b(0, 255, 0);
        // Point p3(parent->real2map_x(x + cos(meas_angle + angle) *
        // measurement), parent->real2map_y(y + sin(meas_angle + angle) *
        // measurement)); vis_map.at<Vec3b>(p3) = Vec3b(255, 0, 0);

        if (scan->ranges[i] > scan->range_max ||
            scan->ranges[i] < scan->range_min)
        {
            continue;
        }
        likelihood *= exp(-pow((estimation - measurement), 2) /
                          pow(LASER_STD, 2) / 2.0) /
                      (LASER_STD * sqrt(2 * M_PI));
    }
    imshow("Localization", vis_map);
    waitKey();

    weight = likelihood;
    return likelihood;
}

Particle::~Particle()
{
}
