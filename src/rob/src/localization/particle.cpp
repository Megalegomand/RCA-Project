#include "particle.h"

using namespace cv;
using namespace sensor_msgs;
using namespace std;

Particle::Particle(float x_, float y_, float angle_, Mat *map_, MCL *parent_)
{
    x = x_;
    y = y_;
    angle = angle_;
    map = map_;
    parent = parent_;
}

bool Particle::outside_bounds()
{
    return x < -REAL_WIDTH / 2 || x > REAL_WIDTH / 2 || y < -REAL_HEIGHT / 2 ||
           y > REAL_HEIGHT / 2;
}

float Particle::wall_distance(float angle)
{
    float x_diff = cos(this->angle + angle);
    float y_diff = sin(this->angle + angle);
    float x_ = parent->real2map_x(x);
    float y_ = parent->real2map_y(y);
    while (true)
    {
        int x_int = x_ + 0.5;
        int y_int = y_ + 0.5;
        if (map->at<Vec3b>(y_int, x_int) == Vec3b(0, 0, 0))
        {
            break;
        }
        x_ += x_diff;
        y_ += y_diff;
    }
    float distance = sqrt(pow(x - parent->map2real_x(x_), 2) +
                          pow(y - parent->map2real_y(y_), 2));

    return sqrt(pow(x - parent->map2real_x(x_), 2) +
                pow(y - parent->map2real_y(y_), 2));
}

void Particle::mark(Mat *vis_map)
{
    if (outside_bounds())
        return;

    // Round to nearest int
    Point p1(parent->real2map_x(x), parent->real2map_y(y));
    Point p2(p1.x + cos(angle) * MARK_DISTANCE,
             p1.y + sin(angle) * MARK_DISTANCE);
    line(*vis_map, p1, p2, Vec3b(255, 0, 0), 1);
    vis_map->at<Vec3b>(p1) = Vec3b(0, 0, 255);
}

double Particle::get_likelihood(const LaserScanConstPtr &scan)
{
    if (outside_bounds())
        return 0;

    double likelihood = 1.0f;
    for (int d = 0; d < LASER_DIVISIONS; d++)
    {
        // ROS_INFO("min: %f, max: %f", scan->angle_min, scan->angle_max);
        int i = d * scan->ranges.size() / LASER_DIVISIONS;
        float meas_angle = scan->angle_min + scan->angle_increment * i;
        float measurement = scan->ranges[i];
        float estimation = wall_distance(meas_angle);

        // Point p2(parent->real2map_x(x + cos(meas_angle + angle) *
        // estimation),
        //         parent->real2map_y(y + sin(meas_angle + angle) *
        //         estimation));
        // vis_map.at<Vec3b>(p2) = Vec3b(0, 255, 0);
        // Point p3(parent->real2map_x(x + cos(meas_angle + angle) *
        // measurement), parent->real2map_y(y + sin(meas_angle + angle) *
        // measurement)); vis_map.at<Vec3b>(p3) = Vec3b(255, 0, 0);

        if (measurement >= scan->range_max)
        {
            measurement = scan->range_max;
        }
        else if (measurement <= scan->range_min)
        {
            measurement = scan->range_min;
        }
        
        if (estimation >= scan->range_max)
        {
            estimation = scan->range_max;
        }
        else if (estimation <= scan->range_min)
        {
            estimation = scan->range_min;
        }

        likelihood *= exp(-pow((estimation - measurement), 2) /
                          pow(LASER_STD, 2) / 2.0) /
                      (LASER_STD * sqrt(2 * M_PI));
    }
    // imshow("Localization", vis_map);
    // waitKey();

    weight = likelihood;
    return likelihood;
}

void Particle::update_pose(float diff_x, float diff_y, float diff_angle)
{
    default_random_engine dre;
    normal_distribution<double> x_dist(diff_x, POS_STD);
    normal_distribution<double> y_dist(diff_y, POS_STD);
    normal_distribution<double> angle_dist(diff_angle, ANGLE_STD);

    x += x_dist(dre);
    y += y_dist(dre);
    angle += angle_dist(dre);
}

Particle::~Particle()
{
}
