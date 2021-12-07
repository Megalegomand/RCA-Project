#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MARK_DISTANCE 3

class Particle
{
private:
    int x;
    int y;
    float angle;
public:
    Particle(int x, int y, float angle);
    void mark(cv::Mat* map);
    ~Particle();
};
