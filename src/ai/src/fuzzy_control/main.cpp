#include "ros/ros.h"
#include "ros/package.h"
#include <ros/console.h>
#include <string>
#include "fuzzy_control.h"

using namespace std;
using namespace ros;

int main(int _argc, char **_argv)
{
    init(_argc, _argv, "fuzzy_control");
    
    FuzzyControl fc();

    spin();

    return 0;
}