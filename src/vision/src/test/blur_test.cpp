#include "ros/ros.h"
#include <gtest/gtest.h>

TEST(TestSuite, testCase2)
{
    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "blur_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}