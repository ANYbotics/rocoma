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
#include <starleth_description/starleth_se_actuator_commands.hpp>

#include <robotUtils/loggers/logger.hpp>
#include <robotUtils/loggers/LoggerStd.hpp>

#include <ros/package.h>

#include <chrono>
#include <cstdint>

NODEWRAP_EXPORT_CLASS(locomotion_controller, locomotion_controller::LocomotionController)


namespace locomotion_controller {

LocomotionController::LocomotionController():
    timeStep_(0.0025),
    time_(0.0),
    model_()
{


}

LocomotionController::~LocomotionController()
{
}

void LocomotionController::init() {

  getNodeHandle().param<double>("controller/time_step", timeStep_, 0.0025);
  std::string loggingScriptFileName = ros::package::getPath("locomotion_controller") + std::string{"/config/logging.script"};
  robotUtils::logger.reset(new robotUtils::LoggerStd);
  robotUtils::LoggerStd* loggerStd = static_cast<robotUtils::LoggerStd*>(robotUtils::logger.get());

  loggerStd->setVerboseLevel(robotUtils::LoggerStd::VL_DEBUG);

  robotUtils::logger->initLogger((int)(1.0/timeStep_), (int)(1.0/timeStep_), 60, loggingScriptFileName);

  robotStateSubscriber_ = subscribe("robot_state", "/robot", 100, &LocomotionController::robotStateCallback, ros::TransportHints().tcpNoDelay());
  joystickSubscriber_ = subscribe("joy", "/joy", 100, &LocomotionController::joystickCallback, ros::TransportHints().tcpNoDelay());

  jointCommandsPublisher_ = advertise<starleth_msgs::SeActuatorCommands>("command_seactuators","/command_seactuators", 100);

  jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
  starleth_description::initializeSeActuatorCommandsForStarlETH(*jointCommands_);

  time_ = 0.0;

  model_.initializeForController(timeStep_);


  // todo: should be true


  model_.getRobotModel()->params().printParams();
  model_.addVariablesToLog();


  controllerManager_.setupControllers(timeStep_, time_, model_.getState(), model_.getCommand());

  switchControllerService_ = getNodeHandle().advertiseService("switch_controller", &ControllerManager::switchController, &this->controllerManager_);
  emergencyStopService_ = advertiseService("emergency_stop", "/emergency_stop", &LocomotionController::emergencyStop);
  resetStateEstimatorClient_ = serviceClient<locomotion_controller_msgs::ResetStateEstimator>("reset_state_estimator", "/reset_state_estimator");
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
    controllerManager_.emergencyStop();
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
