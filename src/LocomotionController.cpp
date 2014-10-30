/*
 * LocomotionController.cpp
 *
 *  Created on: Oct 30, 2014
 *      Author: gech
 */

#include "locomotion_controller/LocomotionController.hpp"

namespace locomotion_controller {

LocomotionController::LocomotionController(ros::NodeHandle& nodeHandle):
    nodeHandle_(nodeHandle)
{

}

LocomotionController::~LocomotionController()
{
}

bool LocomotionController::initialize() {
  std::string robotStateTopicName;
  nodeHandle_.param<std::string>("topic/robot_state", robotStateTopicName, "robot_state");
  robotStateSubscriber_ = nodeHandle_.subscribe(robotStateTopicName, 1, &LocomotionController::robotStateCallback, this);

  std::string joystickTopicName;
  nodeHandle_.param<std::string>("topic/joy", joystickTopicName, "joy");
  joystickSubscriber_ = nodeHandle_.subscribe(joystickTopicName, 1, &LocomotionController::joystickCallback, this);

  std::string jointCommandsTopicName;
  nodeHandle_.param<std::string>("topic/joint_commands", jointCommandsTopicName, "joint_commands");
  jointCommandsPublisher_ = nodeHandle_.advertise<starleth_msgs::SeActuatorCommands>(jointCommandsTopicName, 1);

  jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
  return true;
}

bool LocomotionController::run() {
  ros::spin();
  return true;
}


void LocomotionController::publish()  {
  if(jointCommandsPublisher_.getNumSubscribers() > 0u) {
    jointCommandsPublisher_.publish(jointCommands_);
  }
}
void LocomotionController::robotStateCallback(const starleth_msgs::RobotState::ConstPtr& msg) {
  ros::Time stamp = ros::Time::now();
  for (int i=0; i<jointCommands_->commands.size(); i++) {
    jointCommands_->commands[i].header.stamp = stamp;
  }
  publish();
}

void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {

}

} /* namespace locomotion_controller */
