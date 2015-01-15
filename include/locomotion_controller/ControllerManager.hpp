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

#ifndef LOCOMOTION_CONTROLLER_CONTROLLERMANAGER_HPP_
#define LOCOMOTION_CONTROLLER_CONTROLLERMANAGER_HPP_

#include <ros/ros.h>
#include <locomotion_controller_msgs/SwitchController.h>
#include <locomotion_controller_msgs/EmergencyStop.h>

#include <locomotion_controller/Model.hpp>

#include "robotTask/tasks/TaskRobotBase.hpp"
#include "robotTask/tasks/tasks.hpp"
#include <roco/controllers/ControllerInterface.hpp>
#include <starlethModel/State.hpp>
#include <starlethModel/Command.hpp>

#include <boost/ptr_container/ptr_vector.hpp>

namespace locomotion_controller {

class ControllerManager;
void add_locomotion_controllers(locomotion_controller::ControllerManager* manager, robotModel::State& state, robotModel::Command& command, ros::NodeHandle& nodeHandle);

class ControllerManager
{
 public:
  typedef roco::controllers::ControllerInterface Controller;
  typedef roco::controllers::ControllerInterface* ControllerPtr;
 public:
  ControllerManager();
  virtual ~ControllerManager();

  void updateController();
  void setupControllers(double dt, double time, robotModel::State& state, robotModel::Command& command, ros::NodeHandle& nodeHandle);
  void addController(ControllerPtr controller);
  bool switchController(locomotion_controller_msgs::SwitchController::Request  &req,
                        locomotion_controller_msgs::SwitchController::Response &res);
  bool emergencyStop();
  bool switchControllerAfterEmergencyStop();
  bool isRealRobot() const;
  void setIsRealRobot(bool isRealRobot);
 protected:
  void switchToEmergencyTask();
 protected:
  double time_;
  double timeStep_;
  bool isInitializingTask_;
  boost::ptr_vector<Controller> controllers_;
  ControllerPtr activeController_;
  bool isRealRobot_;
};

} /* namespace locomotion_controller */

#endif /* LOCOMOTION_CONTROLLER_CONTROLLERMANAGER_HPP_ */
