#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "brushfire.h"

using namespace cv;
using namespace std;

int main(int, char **)
{
    Mat image;
    image = imread("/home/megalegomand/OneDrive/Uni/5Semester/ROB/Prg2/nyt.png", IMREAD_COLOR);
    if (image.empty())
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }
    namedWindow("Map");
    imshow("Map", image);

    Mat bf_img = BrushFire::brushfire(&image, Neighbors::Connected8);
    imshow("BF", BrushFire::bf_to_display(&bf_img));

    Mat canny;
    Canny(BrushFire::bf_to_display(&bf_img), canny, 0, 0, 5);
    imshow("Canny", canny);

    imshow("Voronoi", BrushFire::voronoi(&bf_img, Neighbors::Connected8));

    while (waitKey(0) != 'q');
}
