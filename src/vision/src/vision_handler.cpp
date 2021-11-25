#include "vision_handler.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

VisHandler::VisHandler()
{
    camera_sub =
        n.subscribe("/robot/image_raw", 1, &VisHandler::camera_callback, this);
}

void VisHandler::camera_callback(const ImageConstPtr &call_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr =
            cv_bridge::toCvCopy(call_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("Test", cv_ptr->image);
    Mat undist;
    ROS_INFO("Test");

    Mat cam_matrix =
        Mat(3, 3, CV_64FC1, this->cam_matrix.data());
    ROS_INFO("Test");
    undistort(cv_ptr->image, undist, cam_matrix, InputArray(dist_vec));

    ROS_INFO("Test");

    imshow("Unidst", undist);
    waitKey(10);

    return;

    // Image smoothing using gaussian, median, average, and bilateral
    Mat gaus_blurred, median_blurred, avg_blur, bi_blur;

    GaussianBlur(undist, gaus_blurred, Size(5, 5), 0);

    medianBlur(undist, median_blurred, 5);

    bilateralFilter(undist, bi_blur, 9, 75, 75);

    blur(undist, avg_blur, Size(3, 3), Point(1, 1));

    imshow("Median blur", median_blurred);
    imshow("Gaus blur", gaus_blurred);
    imshow("Bilateral blur", bi_blur);
    imshow("Gaus blur", avg_blur);

    Mat canny;
    Canny(gaus_blurred, canny, 10, 20);

    vector<Vec3f> circles;

    cvtColor(gaus_blurred, canny, CV_BGR2GRAY);

    imshow("Lul", canny);

    HoughCircles(canny, circles, HOUGH_GRADIENT, 1, canny.rows / 4, 40, 10, 1,
                 30);

    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // draw the circle center
        circle(cv_ptr->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        // draw the circle outline
        circle(cv_ptr->image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }
    namedWindow("circles", 1);
    imshow("circles", cv_ptr->image);

    waitKey(500);
}

VisHandler::~VisHandler()
{
}
