#include "mcl.h"

using namespace cv;
using namespace std;

MCL::MCL(Mat *map)
{
    map2real_c = REAL_WIDTH / map->cols;

    // rw / mw = m2r = r / m => r = m * m2r
    // rw / mw = m2r = r / m => m = r / m2r

    this->map = map;
    particles = vector<Particle>();

    lidar_sub =
        nh.subscribe("/robot/laser/scan", 1, &MCL::lidar_callback, this);
}

void MCL::randomize_particles()
{
    default_random_engine dre;

    for (int i = 0; i < particle_amount; i++)
    {
        float x, y;

        // Get random coordinate set which is not colliding with wall
        do
        {
            uniform_real_distribution<double> x_dist(-REAL_WIDTH / 2,
                                                     REAL_WIDTH / 2);
            uniform_real_distribution<double> y_dist(-REAL_HEIGHT / 2,
                                                     REAL_HEIGHT / 2);
            x = x_dist(dre);
            y = y_dist(dre);
        } while (map->at<Vec3b>(real2map_y(y), real2map_x(x)) ==
                 Vec3b(0, 0, 0));

        // Random angle
        uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
        float angle = angle_dist(dre);
        particles.push_back(Particle(x, y, angle, map, this));
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

void MCL::lidar_callback(const sensor_msgs::LaserScanConstPtr &scan)
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
        particles[i].norm_weight = weights[i];
    }

    //visualize();

    resample(weights);

    visualize();
}

void MCL::resample(std::vector<double> weights)
{
    default_random_engine dre;
    uniform_real_distribution<double> bin_dist(0.0, 1.0);

    vector<Particle> particles_ = vector<Particle>();
    particles_.clear();

    for (int j = 0; j < particle_amount; j++)
    {
        int i = 0;
        double c = weights[i];
        double bin = bin_dist(dre);

        while (bin > c)
        {
            i++;
            c += weights[i];
        }
        particles_.push_back(particles[i]);
    }

    particles = particles_;
    ROS_INFO("%i", (int)particles.size());
}

float MCL::map2real(float v)
{
    return v * map2real_c;
}

int MCL::real2map(float v)
{
    return v / map2real_c + 0.5;
}

int MCL::real2map_x(float v)
{
    return real2map(v + REAL_WIDTH / 2);
}

int MCL::real2map_y(float v)
{
    return real2map(v + REAL_HEIGHT / 2);
}

float MCL::map2real_x(int v)
{
    return map2real(v) - REAL_WIDTH / 2;
}

float MCL::map2real_y(int v)
{
    return map2real(v) - REAL_HEIGHT / 2;
}

MCL::~MCL()
{
}