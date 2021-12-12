#include "brushfire.h"
#include "ros/package.h"
#include "ros/ros.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

int main(int, char **)
{
    Mat image;
    image = imread(ros::package::getPath("env_sim") +
                       "/models/bigworld/meshes/floor_plan.png",
                   IMREAD_COLOR);
    if (image.empty())
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }
    cv::resize(image, image, cv::Size(), 8.0f, 8.0f, cv::INTER_NEAREST);

    namedWindow("Map");
    imshow("Map", image);

    Mat bf_img = BrushFire::brushfire(&image, Neighbors::Connected8);
    Mat bf_real = BrushFire::bf_to_display(&bf_img);
    imshow("BF", bf_real);
    imwrite(ros::package::getPath("rob") + "/bf.png", bf_real);

    Mat voro = BrushFire::voronoi(&bf_img, Neighbors::Connected8);
    Mat canny;
    Canny(voro, canny, 0, 0, 5);
    imshow("Voronoi", voro);
    imwrite(ros::package::getPath("rob") + "/voro.png", voro);

    while (waitKey(0) != 'q')
        ;
}
