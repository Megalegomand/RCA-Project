#include "vis_handler.h"

using namespace cv;
using namespace std;

VisHandler::VisHandler()
{
    camera_sub =
        n.subscribe("/robot/image_raw", 1, &VisHandler::camera_callback, this);
}

void VisHandler::camera_callback(const sensor_msgs::ImageConstPtr call_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(call_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("Test", cv_ptr->image);
    
    Mat blurred;
    GaussianBlur(cv_ptr->image, blurred, Size(7, 7), 0);

    imshow("Blur", blurred);

    Mat canny;
    Canny(blurred, canny, 10, 20);

    imshow("Lul", canny);

    vector<Vec3f> circles;

    HoughCircles(canny, circles, HOUGH_GRADIENT,
                 2, canny.rows/4, 200, 100 );

    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
    namedWindow( "circles", 1 );
    imshow( "circles", cv_ptr->image );

    waitKey(500);
}

VisHandler::~VisHandler()
{
}
