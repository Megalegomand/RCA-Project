#include "sensor_msgs/Image.h"

class VisHandler
{
private:

public:
    VisHandler();

    void camera_callback(sensor_msgs::ImageConstPtr image);

    ~VisHandler();
};
