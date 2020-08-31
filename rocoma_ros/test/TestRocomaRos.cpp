/*!
 * @file        TestRocomaRos.cpp
 * @authors     Valentin Yuryev (ANYbotics)
 * @brief       Test ROS interface.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

class TestRocomaRos : public ::testing::Test {
  void SetUp() override {
    const std::map<std::string, std::string> remappings{};
    ros::init(remappings, "rocoma_ros");
    ros::start();
  }

  void TearDown() override { ros::shutdown(); }
};

TEST_F(TestRocomaRos, DummyTest) {  // NOLINT
  EXPECT_TRUE(true);
}