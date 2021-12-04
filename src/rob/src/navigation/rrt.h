#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rrt_point.h"

class RRT
{
private:
    cv::Mat map;
public:
    RRT(cv::Mat map, RRTPoint start);
    bool build(RRTPoint end);
    ~RRT();
};
