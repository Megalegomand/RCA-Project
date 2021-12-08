#include "particle.h"

using namespace cv;
using namespace sensor_msgs;

Particle::Particle(float x, float y, float angle)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
}

void Particle::mark(Mat* map)
{
    // Round to nearest int
    Point p1((int) x + 0.5f, (int) y + 0.5f);
    Point p2((int) x + cos(angle) * MARK_DISTANCE + 0.5f, (int) y + sin(angle) * MARK_DISTANCE + 0.5f);
    line(*map, p1, p2, Vec3b(255, 0, 0));
}

float Particle::sensor_update(const LaserScanConstPtr scan)
{
    
}

Particle::~Particle()
{
}
