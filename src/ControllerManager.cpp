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
 * @file    ControllerManager.hpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */
#include "locomotion_controller/ControllerManager.hpp"
#include "locomotion_controller/ControllerRos.hpp"
#include <locomotion_controller/LocomotionController.hpp>

#include "signal_logger_ros/LoggerRos.hpp"

namespace locomotion_controller {

ControllerManager::ControllerManager(locomotion_controller::LocomotionController* locomotionController) :
    timeStep_(0.0),
    isInitializingTask_(false),
    controllers_(),
    activeController_(nullptr),
    isRealRobot_(false),
    locomotionController_(locomotionController)
{

}


ControllerManager::~ControllerManager()
{

}


void ControllerManager::setupControllers(
    double dt,
    quadruped_model::State& state,
    quadruped_model::Command& command,
    boost::shared_mutex& mutexState,
    boost::shared_mutex& mutexCommand,
    ros::NodeHandle& nodeHandle)
{
  timeStep_ = dt;

  /* Create controller freeze, which is active until estimator converged*/
  auto controller = new ControllerRos<robot_controller::RocoFreeze>(state, command, mutexState, mutexCommand);
  controller->setControllerManager(this);
  //controller->setIsCheckingState(false);
  addController(controller);
  activeController_ = &controllers_.back();

  if (!activeController_->initializeController(timeStep_)) {
    ROS_FATAL("Could not initialized NoTask!");
  }

  add_locomotion_controllers(this, state, mutexState, command, mutexCommand, nodeHandle);

  emergencyStopStatePublisher_.shutdown();
  emergencyStopStatePublisher_ = nodeHandle.advertise<any_msgs::State>("notify_emergency_stop", 1, true);
  publishEmergencyState(true);
}

void ControllerManager::addController(ControllerPtr controller)  {
  ROS_INFO("[ControllerManager] Adding controller %s ...", controller->getName().c_str());
  controllers_.push_back(controller);

  if (!controller->createController(timeStep_)) {
    ROS_ERROR("[ControllerManager] Could not create controller %s!", controller->getName().c_str());
    std::string error = "Could not add controller " +  controller->getName() + "!";
    throw std::runtime_error(error);
  }
  ROS_INFO("[ControllerManager] ... finished adding controller %s.", controller->getName().c_str());
}




void ControllerManager::updateController() {
  {
    std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);
    activeController_->advanceController(timeStep_);
  }
}

bool ControllerManager::emergencyStop() {
 activeController_->stopController();
 return true;
}

bool ControllerManager::switchControllerAfterEmergencyStop() {
  this->switchToEmergencyTask();
 return true;
}

void ControllerManager::switchToEmergencyTask() {
  std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);
  if (activeController_->getName() != "Freeze") {
    for (auto& controller : controllers_) {
      if (controller.getName() == "Freeze") {
        activeController_ = &controller;
        activeController_->resetController(timeStep_);
        return;
      }
    }
    throw std::runtime_error("Controller 'freeze' not found!");
  }

}


void ControllerManager::notifyEmergencyState() {
  publishEmergencyState(false);
  publishEmergencyState(true);
}

void ControllerManager::publishEmergencyState(bool isOk) {
  emergencyStopStateMsg_.stamp = ros::Time::now();
  emergencyStopStateMsg_.is_ok = isOk;

  any_msgs::StatePtr stateMsg( new any_msgs::State(emergencyStopStateMsg_) );
  emergencyStopStatePublisher_.publish( any_msgs::StateConstPtr(stateMsg) );
}


bool ControllerManager::switchController(locomotion_controller_msgs::SwitchController::Request  &req,
                                         locomotion_controller_msgs::SwitchController::Response &res)
{

  //--- Check if controller is already active
  if (req.name == activeController_->getName()) {
    res.status = res.STATUS_RUNNING;
    ROS_INFO("Controller is already running!");
    return true;
  }

  for (auto& controller : controllers_) {
    if (req.name == controller.getName()) {

      ControllerPtr initController = &controller;
      ControllerPtr oldActiveController = activeController_;

      initController->initializeController(timeStep_);

      if (initController->isInitialized()) {
       {
          std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);
          activeController_ = initController;
       }

       oldActiveController->swapOut();

       res.status = res.STATUS_SWITCHED;
                ROS_INFO("Switched to controller %s",
                         initController->getName().c_str());
      }
      else {
        // switch to freeze controller
        switchToEmergencyTask();
        res.status = res.STATUS_ERROR;
        ROS_INFO("Could not switch to controller %s", initController->getName().c_str());
      }

      return true;
    }
  }
  res.status = res.STATUS_NOTFOUND;
  ROS_INFO("Controller %s not found!", req.name.c_str());
  return true;
}



bool ControllerManager::getAvailableControllers(locomotion_controller_msgs::GetAvailableControllers::Request &req,
                                                locomotion_controller_msgs::GetAvailableControllers::Response &res)
{

  for (auto& controller : controllers_) {
    res.available_controllers.push_back(controller.getName());
  }

  return true;
}


locomotion_controller::LocomotionController* ControllerManager::getLocomotionController() {
  return locomotionController_;
}


bool ControllerManager::getActiveController(locomotion_controller_msgs::GetActiveController::Request &req,
                                            locomotion_controller_msgs::GetActiveController::Response &res) {

  res.active_controller_name = activeController_->getName();
  res.active_controller_locomotion_mode = activeController_->getLocomotionModeName();

  return true;
}


bool ControllerManager::isRealRobot() const {
	return isRealRobot_;
}
void ControllerManager::setIsRealRobot(bool isRealRobot) {
	isRealRobot_ = isRealRobot;
}

void ControllerManager::cleanup() {
  emergencyStop();
  for (auto& controller : controllers_) {
      controller.cleanupController();
  }
}

} /* namespace locomotion_controller */
