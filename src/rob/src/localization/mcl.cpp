#include "mcl.h"

using namespace cv;
using namespace std;
using namespace tf;

MCL::MCL(Mat *map)
{
    map2real_c = REAL_WIDTH / map->cols;

    // rw / mw = m2r = r / m => r = m * m2r
    // rw / mw = m2r = r / m => m = r / m2r

    this->map = map;
    loc_map = map->clone();
    particles = vector<Particle>();

    prev_real = Point2i(real2map_x(0.0), real2map_y(0.0));
    prev_est = Point2i(real2map_x(0.0), real2map_y(0.0));

    // lidar_sub =
    //    nh.subscribe("/robot/laser/scan", 1, &MCL::lidar_callback, this);

    lidar_sub.subscribe(nh, "/robot/laser/scan", 10);
    odom_sub.subscribe(nh, "/robot/odom", 10);

    sync.reset(new Sync(ParticleSyncPolicy(10), lidar_sub, odom_sub));
    sync->registerCallback(boost::bind(&MCL::callback, this, _1, _2));

    csv.open(ros::package::getPath("rob") + "/local.csv");
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

void MCL::init_particles()
{
    for (int i = 0; i < particle_amount; i++)
    {
        particles.push_back(Particle(0.0, 0.0, 0.0, map, this));
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
    waitKey(10);
}

template <typename T> struct comma_separator : std::numpunct<T>
{
    typename std::numpunct<T>::char_type do_decimal_point() const
    {
        return ',';
    }
};

template <typename T>
std::basic_ostream<T> &comma_sep(std::basic_ostream<T> &os)
{
    os.imbue(std::locale(std::locale(""), new comma_separator<T>));
    return os;
}

void MCL::callback(const sensor_msgs::LaserScanConstPtr &scan,
                   const nav_msgs::OdometryConstPtr &odom)
{
    Quaternion quat;
    quaternionMsgToTF(odom->pose.pose.orientation, quat);
    double a_z, a_y, a_x;
    tf::Matrix3x3(quat).getEulerYPR(a_z, a_y, a_x);

    float new_x = odom->pose.pose.position.x;
    float new_y = odom->pose.pose.position.y;
    float new_angle = a_z;
    float diff_x = new_x - prev_x;
    float diff_y = new_y - prev_y;
    float diff_angle = new_angle - prev_angle;
    ROS_INFO("%f %f %f", diff_x, diff_y, diff_angle);
    prev_x = new_x;
    prev_y = new_y;
    prev_angle = new_angle;

    default_random_engine dre;
    normal_distribution<double> x_dist(diff_x, POS_STD);
    normal_distribution<double> y_dist(diff_y, POS_STD);
    normal_distribution<double> angle_dist(diff_angle, ANGLE_STD);

    // Motion update
    for (Particle &p : particles)
    {
        p.update_pose(x_dist(dre), y_dist(dre), angle_dist(dre));
    }

    // Sensor update
    double weight_sum = 0.0f;
    vector<double> weights;
    for (Particle &p : particles)
    {
        double weight = p.get_likelihood(scan);
        weight_sum += weight;
        weights.push_back(weight);
    }

    float weight_max = 0.0f;
    Particle *best_particle;
    for (int i = 0; i < weights.size(); i++)
    {
        weights[i] *= 1 / weight_sum;
        particles[i].norm_weight = weights[i];
        if (weights[i] > weight_max)
        {
            weight_max = weights[i];
            best_particle = &particles[i];
        }
    }

    gazebo_msgs::GetModelState robot_state;
    robot_state.request.model_name = "pioneer2dx";
    if (ros::service::call("/gazebo/get_model_state", robot_state))
    {
        double dist =
            best_particle->get_dist(robot_state.response.pose.position.x,
                                    robot_state.response.pose.position.y);
        ROS_INFO("dist: %f", dist);
        csv << comma_sep << dist << endl;

        Point2i real = Point2i(real2map_x(robot_state.response.pose.position.x), real2map_y(robot_state.response.pose.position.y));
        Point2i est = Point2i(real2map_x(best_particle->get_x()), real2map_y(best_particle->get_y()));

        line(loc_map, est, prev_est, Vec3b(255,0,0));
        line(loc_map, real, prev_real, Vec3b(0,0,255));
        
        imwrite(ros::package::getPath("rob") + "/local.png", loc_map);

        prev_real = real;
        prev_est = est;
    }

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