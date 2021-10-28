#include "fl/Headers.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <ros/console.h>
#include <string>
#include "std_msgs/String.h"

#include <sstream>

using namespace std;
using namespace fl;

int main(int _argc, char **_argv){
    string src_folder = ros::package::getPath("ai") + "/src";
    ROS_INFO("AI Source folder: %s", src_folder.c_str());
    Engine* engine = FllImporter().fromFile(src_folder + "/fuzzy_logic.fll");

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable* obstacle = engine->getInputVariable("obstacle");
    OutputVariable* steer = engine->getOutputVariable("mSteer");

    for (int i = 0; i <= 50; ++i){
        scalar location = obstacle->getMinimum() + i * (obstacle->range() / 50);
        obstacle->setValue(location);
        engine->process();
        FL_LOG("obstacle.input = " << Op::str(location) << 
            " => " << "steer.output = " << Op::str(steer->getValue()));
    }
}