#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <opencv2/opencv.hpp>

#include <iostream>

static boost::mutex mutex;

int main(int _argc, char **_argv)
{
    cv::Mat im = cv::Mat(100, 100, CV_8UC1, cv::Scalar(0));
    cv::imshow("camera", im);

    ros::init(_argc, _argv, "manual_control");

    ros::NodeHandle n;

    ros::Publisher movement_publisher = n.advertise<geometry_msgs::Twist>("/robot/control", 100);

    ros::Rate loop_rate(100);

    const int key_left = 81;
    const int key_up = 82;
    const int key_down = 84;
    const int key_right = 83;
    const int key_esc = 27;

    float speed = 0.0;
    float dir = 0.0;


    while (ros::ok())
    {
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        if (key == key_esc)
            break;

        if ((key == key_up) && (speed <= 1.2f))
            speed += 0.05;
        else if ((key == key_down) && (speed >= -1.2f))
            speed -= 0.05;
        else if ((key == key_right) && (dir <= 0.4f))
            dir += 0.05;
        else if ((key == key_left) && (dir >= -0.4f))
            dir -= 0.05;
        else
        {
            dir *= 0.95;
            speed *= 0.95;
        }

        geometry_msgs::Twist msg = geometry_msgs::Twist();

        msg.linear.x = speed;
        msg.angular.z = dir;

        movement_publisher.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
