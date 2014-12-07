/*
 * Copyright (c) 2014, Christian Gehring
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL  Christian Gehring BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/*!
 * @file    LocomotionController.cpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */

#include <pluginlib/class_list_macros.h>

#include "locomotion_controller/LocomotionController.hpp"

#include "robotUtils/terrains/TerrainPlane.hpp"

#include <ros/callback_queue.h>

#include <locomotion_controller_msgs/ResetStateEstimator.h>

#include <starlethModel/starleth/starleth.hpp>

#include <chrono>
#include <cstdint>

NODEWRAP_EXPORT_CLASS(locomotion_controller, locomotion_controller::LocomotionController)


namespace locomotion_controller {

LocomotionController::LocomotionController():
    timeStep_(0.0025),
    time_(0.0),
    model_(),
    state_(),
    command_()
{


}

LocomotionController::~LocomotionController()
{
}

void LocomotionController::init() {

  getNodeHandle().param<double>("controller/time_step", timeStep_, 0.0025);

  robotStateSubscriber_ = subscribe("robot_state", "/robot", 100, &LocomotionController::robotStateCallback, ros::TransportHints().tcpNoDelay());
  joystickSubscriber_ = subscribe("joy", "/joy", 100, &LocomotionController::joystickCallback, ros::TransportHints().tcpNoDelay());

  ros::AdvertiseOptions opSea;
  opSea.init<starleth_msgs::SeActuatorCommands>("command_seactuators", 100);
  jointCommandsPublisher_ = advertise("command_seactuators",opSea);

  jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
  for (int i=0; i<jointCommands_->commands.size(); i++) {
    jointCommands_->commands[i].mode =  jointCommands_->commands[i].MODE_MOTOR_VELOCITY;
    jointCommands_->commands[i].motorVelocity = 0.0;
  }

  time_ = 0.0;

  model_.initializeForController(timeStep_);

  state_.setRobotModelPtr(model_.getRobotModel());
  state_.setTerrainPtr(model_.getTerrainModel());
  command_.setRobotModelPtr(model_.getRobotModel());
  robotModel::initializeStateForStarlETH(state_);
  robotModel::initializeCommandForStarlETH(command_);

  model_.getRobotModel()->params().printParams();


  controllerManager_.setupControllers(timeStep_, time_, state_, command_);

  switchControllerService_ = getNodeHandle().advertiseService("switch_controller", &ControllerManager::switchController, &this->controllerManager_);
  ros::AdvertiseServiceOptions defaultOptions;

  defaultOptions.init<locomotion_controller_msgs::EmergencyStop::RequestType, locomotion_controller_msgs::EmergencyStop::ResponseType>("emergency_stop", boost::bind(&LocomotionController::emergencyStop, this, _1, _2));
  emergencyStopService_ = advertiseService("emergency_stop", defaultOptions);
//  emergencyStopService_ = advertiseService("emergency_stop", &ControllerManager::emergencyStop, &this->controllerManager_);


  std::string ns = std::string("clients/")+std::string("reset_state_estimator");
  std::string serviceResetStateEstimator;
  getNodeHandle().param(ns+"/service", serviceResetStateEstimator, std::string("reset_state_estimator"));
  resetStateEstimatorClient_ = getNodeHandle().serviceClient<locomotion_controller_msgs::ResetStateEstimator>(serviceResetStateEstimator);
}

bool LocomotionController::run() {
//  ros::Rate loop_rate(800);
//  while (ros::ok())
//  {
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.0));
////    ros::spinOnce();
////    loop_rate.sleep();
//  }
  ros::spin();
  return true;
}


void LocomotionController::publish()  {

  model_.getSeActuatorCommands(jointCommands_);


  if(jointCommandsPublisher_.getNumSubscribers() > 0u) {
    jointCommandsPublisher_.publish(jointCommands_);
    ros::spinOnce();
  }

}
void LocomotionController::robotStateCallback(const starleth_msgs::RobotState::ConstPtr& msg) {
//  ROS_INFO("Received state");
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  start = std::chrono::steady_clock::now();

  model_.setRobotState(msg);
  controllerManager_.updateController();

  publish();

  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();

  int64_t timeStep = (int64_t)(timeStep_*1e9);
  if (elapsedTimeNSecs > timeStep) {
    ROS_INFO("Warning: computation is not real-time! Elapsed time: %lf ms\n", (double)elapsedTimeNSecs*1e-6);
  }

  time_ += timeStep_;
}

void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {

  model_.setJoystickCommands(msg);

  // START + LF buttons
  if (msg->buttons[4] == 1 && msg->buttons[7] == 1 ) {
    locomotion_controller_msgs::SwitchController::Request  req;
    locomotion_controller_msgs::SwitchController::Response res;
    req.name = "LocoDemo";
    if(!controllerManager_.switchController(req,res)) {
    }
    ROS_INFO("Switched task by joystick (status: %d)",res.status);

  }
  // RB button
  if (msg->buttons[5] == 1 ) {
    NODEWRAP_INFO("Emergency stop by joystick!");
    locomotion_controller_msgs::EmergencyStop::Request  req;
    locomotion_controller_msgs::EmergencyStop::Response res;
    emergencyStop(req, res);

  }
}

bool LocomotionController::emergencyStop(locomotion_controller_msgs::EmergencyStop::Request  &req,
                                         locomotion_controller_msgs::EmergencyStop::Response &res) {

  bool result = true;

  //---
  if(!controllerManager_.emergencyStop()) {
    result = false;
  }
  //---

  //---  Reset estimator.
  if (resetStateEstimatorClient_.exists()) {
    locomotion_controller_msgs::ResetStateEstimator resetEstimatorService;
    if(!resetStateEstimatorClient_.call(resetEstimatorService)) {
      result = false;
    }
  }
  //---

  return result;
}




} /* namespace locomotion_controller */
