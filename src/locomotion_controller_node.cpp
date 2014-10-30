
#include <ros/ros.h>
#include "locomotion_controller/LocomotionController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locomotion_controller");
  ros::NodeHandle nodeHandle("~");
  locomotion_controller::LocomotionController locomotion_controller(nodeHandle);
  locomotion_controller.initialize();
  locomotion_controller.run();
  return 0;
}
