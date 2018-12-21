/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Main test file
 */

#include <gtest/gtest.h>
#include <ros/time.h>

using ::testing::InitGoogleTest;

/* RUN TESTS */
int main(int argc, char** argv) {
  InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
