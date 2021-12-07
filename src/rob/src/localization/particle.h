#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Particle
{
private:
    int x;
    int y;
    float angle;
public:
    Particle(int x, int y, float angle);
    ~Particle();
};
