/*
 * LocomotionController.hpp
 *
 *  Created on: Oct 30, 2014
 *      Author: gech
 */

#ifndef LOCOMOTIONCONTROLLER_HPP_
#define LOCOMOTIONCONTROLLER_HPP_

#include <ros/ros.h>
#include <starleth_msgs/RobotState.h>
#include <sensor_msgs/Joy.h>
#include <starleth_msgs/SeActuatorCommands.h>

#include <memory>

namespace locomotion_controller {

class LocomotionController
{
 public:
  LocomotionController(ros::NodeHandle& nodeHandle);
  virtual ~LocomotionController();

  bool initialize();
  bool run();


 protected:
  void publish();
  void robotStateCallback(const starleth_msgs::RobotState::ConstPtr& msg);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
 private:
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber robotStateSubscriber_;
  ros::Subscriber joystickSubscriber_;
  ros::Publisher jointCommandsPublisher_;
  starleth_msgs::SeActuatorCommandsPtr jointCommands_;
};

} /* namespace locomotion_controller */

#endif /* LOCOMOTIONCONTROLLER_HPP_ */
