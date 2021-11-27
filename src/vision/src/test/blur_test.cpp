#include "ros/ros.h"
#include <gtest/gtest.h>
#include "sensor_msgs/Image.h"

using namespace ros;
using namespace testing;

TEST(TestSuite, testCase2)
{
    NodeHandle n;
    ImageConstPtr img_raw ros::topic::waitForMessage<sensor_msgs::Image>("/robot/image_raw", n, DURATION_MAX);

    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  InitGoogleTest(&argc, argv);
  init(argc, argv, "blur_test");
  return RUN_ALL_TESTS();
}