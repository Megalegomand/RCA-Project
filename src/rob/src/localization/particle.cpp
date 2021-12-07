#include "particle.h"

using namespace cv;

Particle::Particle(int x, int y, float angle)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
}

void Particle::mark(Mat* map)
{
    Point p1(x, y);
    Point p2(x + cos(angle) * MARK_DISTANCE, y + sin(angle) * MARK_DISTANCE);
    line(*map, p1, p2, Vec3b(255, 0, 0));
}

Particle::~Particle()
{
}
