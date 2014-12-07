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


namespace locomotion_controller {

ControllerManager::ControllerManager() :
    time_(0.0),
    timeStep_(0.0),
    isInitializingTask_(false),
    controllers_(),
    activeController_(nullptr)
{

}

ControllerManager::~ControllerManager()
{
}

void ControllerManager::setupControllers(double dt, double time, robotModel::State& state, robotModel::Command& command)  {
  time_ = time;
  timeStep_ = dt;

  /* Create no task, which is active until estimator converged*/
  addController(new ControllerRos<robotTask::NoTaskRos>(state, command));
  activeController_ = &controllers_.back();
  if (!activeController_->initializeController(timeStep_)) {
    ROS_FATAL("Could not initialized NoTask!");
  }

  add_locomotion_controllers(this, state, command);

}

void ControllerManager::addController(ControllerPtr controller)  {
  controllers_.push_back(controller);
//  controller = &controllers_.back();


  ROS_INFO("Added Task %s.", controller->getName().c_str());
  if (!controller->createController(timeStep_)) {
    std::string error = "Could not add controller " +  controller->getName() + "!";
    throw std::runtime_error(error);
  }
}




void ControllerManager::updateController() {
  activeController_->advanceController(timeStep_);
}

bool ControllerManager::emergencyStop() {
 locomotion_controller_msgs::SwitchController::Request req;
 locomotion_controller_msgs::SwitchController::Response res;
 req.name = "EmergencyStop";
 return switchController(req, res);
}


bool ControllerManager::switchController(locomotion_controller_msgs::SwitchController::Request  &req,
                                         locomotion_controller_msgs::SwitchController::Response &res)
{
  std::string reqTaskName = req.name;
  if (req.name == "EmergencyStop") {
    ROS_INFO("Emergency Stop!");

    reqTaskName = "No Task";
  }

  //--- Check if controller is already active
  if (reqTaskName == activeController_->getName()) {
    res.status = res.STATUS_RUNNING;
    ROS_INFO("Controller is already running!");
    return true;
  }

  for (auto& controller : controllers_) {
    if (reqTaskName == controller.getName()) {
      activeController_ = &controller;
      res.status = res.STATUS_SWITCHED;
//      isInitializingTask_ = true;
      if (!activeController_->initializeController(timeStep_)) {
        throw std::runtime_error("Could not initialize the task!");
      }

      ROS_INFO("Switched to controller %s", activeController_->getName().c_str());
      return true;
    }
  }
  res.status = res.STATUS_NOTFOUND;
  ROS_INFO("Controller %s not found!", reqTaskName.c_str());
  return true;
}



} /* namespace locomotion_controller */
