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
#include <string>

NODEWRAP_EXPORT_CLASS(locomotion_controller, locomotion_controller::LocomotionController)


namespace locomotion_controller {

LocomotionController::LocomotionController():
    timeStep_(0.0025),
    isRealRobot_(false),
    model_(),
    controllerManager_()
{


}

LocomotionController::~LocomotionController()
{
}

void LocomotionController::init() {
  //--- Read parameters.
  getNodeHandle().param<double>("controller/time_step", timeStep_, 0.0025);
  getNodeHandle().param<bool>("controller/is_real_robot", isRealRobot_, false);

  //---

  //--- Configure logger.
  std::string loggingScriptFilename;
  getNodeHandle().param<std::string>("logger/script", loggingScriptFilename, "");
  if (loggingScriptFilename.empty()){
    loggingScriptFilename = ros::package::getPath("locomotion_controller") + std::string{"/config/logging.script"};
  }
  double samplingTime;
  getNodeHandle().param<double>("logger/sampling_time", samplingTime, 60.0);
  NODEWRAP_INFO("Initialize logger with sampling time: %lfs and script: %s.", samplingTime, loggingScriptFilename.c_str());
  robotUtils::logger.reset(new robotUtils::LoggerStd);
  robotUtils::LoggerStd* loggerStd = static_cast<robotUtils::LoggerStd*>(robotUtils::logger.get());
  loggerStd->setVerboseLevel(robotUtils::LoggerStd::VL_DEBUG);
  robotUtils::logger->initLogger((int)(1.0/timeStep_), (int)(1.0/timeStep_), samplingTime, loggingScriptFilename);
  //---

  //--- Configure controllers
  {
    std::lock_guard<std::mutex> lockControllerManager(mutexModelAndControllerManager_);

    // Initialize robot and terrain models
    model_.initializeForController(timeStep_,isRealRobot_);
    model_.getRobotModel()->params().printParams();
    model_.addVariablesToLog();


    controllerManager_.setIsRealRobot(isRealRobot_);
    controllerManager_.setupControllers(timeStep_, model_.getState(), model_.getCommand());
  }
  //---

  initializeMessages();
  initializeServices();
  initializePublishers();
  initializeSubscribers();
}

void LocomotionController::cleanup() {

}

void LocomotionController::initializeMessages() {
  //--- Initialize joint commands.
  {
    std::lock_guard<std::mutex> lock(mutexJointCommands_);
    jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
    starleth_description::initializeSeActuatorCommandsForStarlETH(*jointCommands_);
  }
  //---
}

void LocomotionController::initializeServices() {
  switchControllerService_ = getNodeHandle().advertiseService("switch_controller", &ControllerManager::switchController, &this->controllerManager_);
  getAvailableControllersService_ = getNodeHandle().advertiseService("get_active_controllers", &ControllerManager::getAvailableControllers, &this->controllerManager_);
  emergencyStopService_ = advertiseService("emergency_stop", "/emergency_stop", &LocomotionController::emergencyStop);
  resetStateEstimatorClient_ = serviceClient<locomotion_controller_msgs::ResetStateEstimator>("reset_state_estimator", "/reset_state_estimator");
}

void LocomotionController::initializePublishers() {
  jointCommandsPublisher_ = advertise<starleth_msgs::SeActuatorCommands>("command_seactuators","/command_seactuators", 100);
}

void LocomotionController::initializeSubscribers() {
  joystickSubscriber_ = subscribe("joy", "/joy", 100, &LocomotionController::joystickCallback, ros::TransportHints().tcpNoDelay());
  commandVelocitySubscriber_ = subscribe("command_velocity", "/command_velocity", 100, &LocomotionController::commandVelocityCallback, ros::TransportHints().tcpNoDelay());
  //--- temporary
  mocapSubscriber_ = subscribe("mocap", "mocap", 100, &LocomotionController::mocapCallback, ros::TransportHints().tcpNoDelay());
  seActuatorStatesSubscriber_ = subscribe("actuator_states", "/actuator_states", 100, &LocomotionController::seActuatorStatesCallback, ros::TransportHints().tcpNoDelay());
  //---

  // this should be last since it will start the controller loop
  robotStateSubscriber_ = subscribe("robot_state", "/robot", 100, &LocomotionController::robotStateCallback, ros::TransportHints().tcpNoDelay());

}



void LocomotionController::publish()  {

  if(jointCommandsPublisher_.getNumSubscribers() > 0u) {
    std::lock_guard<std::mutex> lock(mutexJointCommands_);
    {
      std::lock_guard<std::mutex> lockControllerManager(mutexModelAndControllerManager_);
      model_.getSeActuatorCommands(jointCommands_);
    }

    starleth_msgs::SeActuatorCommandsConstPtr jointCommands(new starleth_msgs::SeActuatorCommands(*jointCommands_));
    jointCommandsPublisher_.publish(jointCommands);
    //ros::spinOnce(); // todo: required?
  }

}
void LocomotionController::robotStateCallback(const starleth_msgs::RobotState::ConstPtr& msg) {
  updateControllerAndPublish(msg);
}

void LocomotionController::updateControllerAndPublish(const starleth_msgs::RobotState::ConstPtr& robotState) {
  //-- Start measuring computation time.
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  start = std::chrono::steady_clock::now();
  //---

  NODEWRAP_DEBUG("Update locomotion controller.");

  {
    std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
    model_.setRobotState(robotState);
    controllerManager_.updateController();
  }
  publish();



  //-- Measure computation time.
  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();
  int64_t timeStep = (int64_t)(timeStep_*1e9);
  if (elapsedTimeNSecs > timeStep) {
    NODEWRAP_WARN("Computation of locomotion controller is not real-time! Elapsed time: %lf ms\n", (double)elapsedTimeNSecs*1e-6);
  }
  if (elapsedTimeNSecs > timeStep*10) {
      NODEWRAP_ERROR("Computation took more than 10 times the maximum allowed computation time (%lf ms)!", timeStep_*1e-3);
      std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
      controllerManager_.emergencyStop();
  }

  //---
}

void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutexJoystick_);
  std::lock_guard<std::mutex> lockControllerManager(mutexModelAndControllerManager_);
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

  //--- Stop the controller.
  {
    std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
    if(!controllerManager_.emergencyStop()) {
      result = false;
    }
  }
  //---

  //---  Reset the estimator.
  if (resetStateEstimatorClient_.exists()) {
    locomotion_controller_msgs::ResetStateEstimator resetEstimatorService;
    if(!resetStateEstimatorClient_.call(resetEstimatorService)) {
      result = false;
    }
  }
  //---

  return result;
}

void LocomotionController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
	model_.setCommandVelocity(msg);
}

void LocomotionController::mocapCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
  model_.setMocapData(msg);
}

void LocomotionController::seActuatorStatesCallback(const starleth_msgs::SeActuatorStates::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
  model_.setSeActuatorStates(msg);
}

} /* namespace locomotion_controller */
