#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rrt_point.h"
#include "vector"

class RRT
{
private:
    cv::Mat* map;
    cv::Mat map_vis;
    std::vector<RRTPoint> nodes;
    int iterations = 100;

    RRTPoint random_point();
    bool exists(RRTPoint& point);
    RRTPoint* closest(RRTPoint& p);

    void init(cv::Mat& map, RRTPoint start);
public:
    RRT(cv::Mat& map, RRTPoint start);
    RRT(cv::Mat& map);
    bool build(RRTPoint end);
    
    ~RRT();
};
