#include "vision_handler.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace ros::topic;

VisHandler::VisHandler()
{
    n = ros::NodeHandle();

    // Get camera info
    CameraInfoConstPtr cam_info =
        waitForMessage<CameraInfo>("/robot/camera_info", n);

    // Make sure it exists
    if (!cam_info)
    {
        ROS_ERROR("No camera info found");
    }
    else
    {
        // Update camera parameters
        camera_matrix = Mat(3, 3, CV_64FC1);
        for (int i = 0; i < 9; i++)
        {
            double *data = (double *)camera_matrix.data;
            data[i] = cam_info->K[i];
        }

        distortion_vector = Mat(cam_info->D.size(), 1, CV_64FC1);
        for (int i = 0; i < cam_info->D.size(); i++)
        {
            double *data = (double *)distortion_vector.data;
            data[i] = cam_info->D[i];
        }

        // Get image from Gazebo
        camera_sub = n.subscribe("/robot/image_raw", 1,
                                 &VisHandler::camera_callback, this);
    }
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

    Mat img = cv_ptr->image.clone();

    imshow("Test12", img);
    /*imwrite("/home/philip/Pictures/Test.png",img);
            return;*/

    bool undistort_bool = false;
    if (n.getParam("/vision_node/undistort", undistort_bool))
    {
        if (undistort_bool)
        {
            Mat undistort_img;
            undistort(img, undistort_img, camera_matrix, distortion_vector);
            img = undistort_img;

            imshow("Unidst", img);
            /*imwrite("/home/philip/Pictures/Undistorted.png",img);
            return;*/
            
        }
    }

    string blur_type;
    int blur_size = 0;
    if (n.getParam("/vision_node/blur_type", blur_type) &&
        n.getParam("/vision_node/blur_size", blur_size))
    {
        Mat blur_img;
        if (blur_type == "gaussian")
        {
            GaussianBlur(img, blur_img, Size(blur_size, blur_size), 0);
            img = blur_img;
        }
        else if (blur_type == "median")
        {
            medianBlur(img, blur_img, blur_size);
            img = blur_img;
        }
        else if (blur_type == "average")
        {
            blur(img, blur_img, Size(blur_size, blur_size));
            img = blur_img;
        }
        else
        {
        }

        imshow("Blur", img);
        //imwrite("/home/philip/Pictures/7x7 matrix.png",img);
        //return;
    }

    // Turn img gray for Hough
    cvtColor(img, img, CV_BGR2GRAY);
    /*vector<Mat> channels;
    split(img,channels);*/

    vector<Vec3f> circles;
    double known_radius = 0.5;
    string distancestring;
    double distance;
    HoughCircles(img, circles, HOUGH_GRADIENT, 1, img.cols / 4, 35, 25,
                 img.cols/16, 7/8* img.cols);

    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // draw the circle center
        circle(img, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        // draw the circle outline
        circle(img, center, radius, Scalar(0, 255, 255), 3, 8, 0);
        distance = (2 * known_radius * camera_matrix.at<double>(1, 1)) / (radius * 2);
        distancestring = to_string(distance);
       /* putText(img,               //target image
                distancestring,              //text
                center, //center of circle
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(118, 185, 0), //font color
                2);*/
    }
    namedWindow("circles", 1);

    imshow("circles", img);
    imwrite("/home/philip/Pictures/Test param1v3.png",img);

    waitKey(500);
}

VisHandler::~VisHandler()
{
}
