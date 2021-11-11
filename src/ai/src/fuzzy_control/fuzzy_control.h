#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <algorithm>
#include "fuzzylite_wrapper.h"

using namespace std;
using namespace ros;

class FuzzyControl
{
private:
    NodeHandle n;
    Subscriber lidar_sub;
    Publisher movement_pub;

    FuzzyLiteWrapper fl_wrapper;

    geometry_msgs::Twist rel_target;
public:
    /**
     * @brief Construct a new Fuzzy Control object
     */
    FuzzyControl();

    /**
     * @brief Lidar callback function
     * 
     * @param scan Callback variable
     */
    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr scan);

    /**
     * @brief Destroy the Fuzzy Control object
     */
    ~FuzzyControl();
};
