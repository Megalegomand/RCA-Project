#include "mcl.h"

using namespace cv;
using namespace std;

MCL::MCL(Mat *map)
{
    this->map = map;
    particles = vector<Particle>();
}

void MCL::randomize_particles()
{
    for (int i = 0; i < particle_amount; i++)
    {
        srand(clock());
        float x, y;

        // Get random coordinate set which is not colliding with wall
        do
        {
            x = static_cast<float>(rand()) /
                      (static_cast<float>(1.0f * RAND_MAX / map->cols));
            y = static_cast<float>(rand()) /
                      (static_cast<float>(1.0f * RAND_MAX / map->rows));
        } while (map->at<Vec3b>((int) y + 0.5f, (int) x + 0.5f) == Vec3b(0, 0, 0));

        // Random angle
        float angle = static_cast<float>(rand()) /
                      (static_cast<float>(RAND_MAX / (2 * M_PI)));
        particles.push_back(Particle(x, y, angle));
    }
}

void MCL::visualize()
{
    Mat vis = map->clone();
    for (Particle p : particles)
    {
        p.mark(&vis);
    }
    imshow("Localization", vis);
}

MCL::~MCL()
{
}