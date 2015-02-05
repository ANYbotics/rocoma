/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Christian Gehring
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
* @file     ControllerRos.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once


#include <roco/time/Time.hpp>
#include <roco/time/TimeStd.hpp>

#include <roco/controllers/ControllerAdapterInterface.hpp>
#include <robotUtils/loggers/logger.hpp>

#include "locomotion_controller/ControllerManager.hpp"

#include <iostream>
#include <exception>      // std::exception

namespace locomotion_controller {


template<typename Controller_>
class ControllerRos:  public roco::controllers::ControllerAdapterInterface, public Controller_
{
 public:
  typedef Controller_ Controller;
  typedef typename Controller::State State;
  typedef typename Controller::Command Command;
 public:
  ControllerRos(State& state, Command& command);
  virtual ~ControllerRos();

  void setControllerManager(ControllerManager* controllerManager);


  virtual bool createController(double dt);
  virtual bool initializeController(double dt);
  virtual bool resetController(double dt);
  virtual bool advanceController(double dt);
  virtual bool changeController();
  virtual bool cleanupController();
  virtual bool stopController();

  virtual const roco::time::Time& getTime() const;
  virtual void setTime(const roco::time::Time& time);
  virtual bool isCheckingCommand() const;
  virtual void setIsCheckingCommand(bool isChecking);
  virtual bool isCheckingState() const;
  virtual void setIsCheckingState(bool isChecking);
  virtual bool isRealRobot() const;
  virtual void setIsRealRobotFromManager(bool isRealRobot);
  virtual void setIsRealRobot(bool isRealRobot);
  virtual const State& getState() const;
  virtual const Command& getCommand() const;
  virtual Command& getCommand();

  virtual void emergencyStop();
 protected:
  bool updateState(double dt);
  bool updateCommand(double dt, bool forceSendingControlModes);
  void sendEmergencyCommand();

 private:
  //! Indicates if the real robot is controller or only a simulated version.
  bool isRealRobot_;
  bool isCheckingCommand_;
  bool isCheckingState_;
  roco::time::TimeStd time_;
  State& state_;
  Command& command_;
  ControllerManager* controllerManager_;
  std::string emergencyStopControllerName_;
};


} // namespace locomotion_controller


#include "ControllerRos.tpp"
