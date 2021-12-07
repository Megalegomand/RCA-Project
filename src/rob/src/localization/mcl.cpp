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
        int x, y;
        do
        {
            x = rand() % map->cols;
            y = rand() % map->rows;
            map->at<Vec3b>(y, x);
        } while (map->at<Vec3b>(y, x) == Vec3b(0, 0, 0));
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