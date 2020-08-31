/*!
 * @file        TestRocomaPlugin.cpp
 * @authors     Valentin Yuryev (ANYbotics)
 * @brief       Test ROS interface.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

class TestRocomaPlugin : public ::testing::Test {
  void SetUp() override {
    const std::map<std::string, std::string> remappings{};
    ros::init(remappings, "rocoma_plugin");
    ros::start();
  }

  void TearDown() override { ros::shutdown(); }
};

TEST_F(TestRocomaPlugin, DummyTest) {  // NOLINT
  EXPECT_TRUE(true);
}