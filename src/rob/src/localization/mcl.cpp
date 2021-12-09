#include "mcl.h"

using namespace cv;
using namespace std;

MCL::MCL(Mat *map)
{
    this->map = map;
    particles = vector<Particle>();

    lidar_sub = nh.subscribe("/robot/laser/scan", 10, &MCL::lidar_callback, this);
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
        particles.push_back(Particle(x, y, angle, map));
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
    waitKey();
}

void MCL::lidar_callback(const sensor_msgs::LaserScanConstPtr& scan)
{
    double weight_sum = 0.0f;
    vector<double> weights;
    for (Particle p : particles)
    {
        double weight = p.get_likelihood(scan);
        weight_sum += weight;
        weights.push_back(weight);
    }
    for (int i = 0; i < weights.size(); i++)
    {
        weights[i] *= 1 / weight_sum;
    }

    resample(weights);

    visualize();
}

void MCL::resample(std::vector<double> weights)
{
    srand(clock());

    vector<Particle> particles_ = vector<Particle>();
    particles_.clear();
    double delta = static_cast<float>(rand()) /
                      (static_cast<float>(1.0f * RAND_MAX / (1 / particle_amount)));
    double c = weights[0];
    int i = 0;

    for (int j = 0; j < particle_amount; j++)
    {
        double u = delta + j * 1.0f / particle_amount;
        while (u > c)
        {
            i++;
            c += weights[i];
        }
        particles_.push_back(particles[i]);
    }

    particles = particles_;
}

MCL::~MCL()
{
}