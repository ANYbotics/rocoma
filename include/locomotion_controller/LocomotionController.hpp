/*
 * LocomotionController.hpp
 *
 *  Created on: Oct 30, 2014
 *      Author: gech
 */

#ifndef LOCOMOTIONCONTROLLER_HPP_
#define LOCOMOTIONCONTROLLER_HPP_

#include <ros/ros.h>

namespace locomotion_controller {

class LocomotionController
{
 public:
  LocomotionController(ros::NodeHandle& nodeHandle);
  virtual ~LocomotionController();

 private:
  ros::NodeHandle& nodeHandle_;
};

} /* namespace locomotion_controller */

#endif /* LOCOMOTIONCONTROLLER_HPP_ */
