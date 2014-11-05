
#include <ros/ros.h>
#include <roscpp_nodewrap/Node.h>
#include "locomotion_controller/LocomotionController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locomotion_controller");
  nodewrap::Node<locomotion_controller::LocomotionController> node;
  node.run();
  return 0;
}
