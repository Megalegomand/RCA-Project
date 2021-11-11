#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/GetModelState.h"
#include "robot/"

bool position_callback(robot::Position::Request& req, robot::Position::Response& res)
{
    gazebo_msgs::GetModelState model_state;
    model_state.request.model_name = "";
    if (ros::service::call("/gazebo/get_model_state", model_state))
    {
        ROS_INFO("%s", model_state.response.pose);
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_node");

    ros::spinOnce();

    return 0;
}
