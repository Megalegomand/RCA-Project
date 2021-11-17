#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/GetModelState.h"
#include "robot/GetPose.h"

bool position_callback(robot::GetPose::Request& req, robot::GetPose::Response& res)
{
    gazebo_msgs::GetModelState model_state;
    model_state.request.model_name = "pioneer2dx";
    if (ros::service::call("/gazebo/get_model_state", model_state))
    {
        res.pose = model_state.response.pose;
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("robot/position", position_callback);

    ros::spin();

    return 0;
}
