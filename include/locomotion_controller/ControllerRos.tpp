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
 * @file     ControllerRos.tpp
 * @author   Christian Gehring
 * @date     Dec, 2014
 * @brief
 */

namespace locomotion_controller {

template<typename Controller_>
ControllerRos<Controller_>::ControllerRos(State& state, Command& command)
    : Controller(),
      isRealRobot_(false),
      isCheckingCommand_(true),
      isCheckingState_(true),
      time_(),
      state_(state),
      command_(command),
      controllerManager_(nullptr),
      emergencyStopControllerName_("Freeze")
{
  if (isRealRobot_) {
    isCheckingCommand_ = true;
    isCheckingState_ = true;
  }
  if (this->getName() == emergencyStopControllerName_) {
    isCheckingState_ = false;
    isCheckingCommand_ = true;
  }

}
;

template<typename Controller_>
ControllerRos<Controller_>::~ControllerRos()
{
  if (!cleanupController()) {
    ROCO_WARN("Could not cleanup controller!");
  }
}
;

template<typename Controller_>
const roco::time::Time& ControllerRos<Controller_>::getTime() const
{
  return static_cast<const roco::time::Time&>(time_);
}

template<typename Controller_>
void ControllerRos<Controller_>::setTime(const roco::time::Time& time)
{
  time_ = time;
}

template<typename Controller_>
bool ControllerRos<Controller_>::isCheckingCommand() const
{
  return isCheckingCommand_;
}

template<typename Controller_>
void ControllerRos<Controller_>::setIsCheckingCommand(bool isChecking)
{
/*  if (isRealRobot_ && !isChecking) {
    ROCO_WARN(
        "Cannot deactivate checking the commands when the real robot is controlled!");
  } else {
    isCheckingCommand_ = isChecking;
  }*/

  if (isRealRobot_ && !isChecking) {
    ROCO_WARN(
        "Checking the command was deactivated!");
  }
  isCheckingCommand_ = isChecking;
}

template<typename Controller_>
bool ControllerRos<Controller_>::isCheckingState() const
{
  return isCheckingState_;
}

template<typename Controller_>
void ControllerRos<Controller_>::setIsCheckingState(bool isChecking)
{
  if (isRealRobot_ && !isChecking) {
    ROCO_WARN(
        "Checking the robot state was deactivated!");
  }
/*  else {
    isCheckingState_ = isChecking;
  }*/
  isCheckingState_ = isChecking;
}

template<typename Controller_>
bool ControllerRos<Controller_>::isRealRobot() const
{
  return isRealRobot_;
}

template<typename Controller_>
void ControllerRos<Controller_>::setIsRealRobotFromManager(bool isRealRobot)
{
  isRealRobot_ = isRealRobot;
}

template<typename Controller_>
void ControllerRos<Controller_>::setIsRealRobot(bool isRealRobot)
{
  ROCO_WARN("You are not allowed to change this option!");
}

template<typename Controller_>
const typename ControllerRos<Controller_>::State& ControllerRos<Controller_>::getState() const
{
  return state_;
}

template<typename Controller_>
const typename ControllerRos<Controller_>::Command& ControllerRos<Controller_>::getCommand() const
{
  return command_;
}

template<typename Controller_>
typename ControllerRos<Controller_>::Command& ControllerRos<Controller_>::getCommand()
{
  return command_;
}

template<typename Controller_>
bool ControllerRos<Controller_>::createController(double dt)
{
  if (this->isCreated()) {
    ROCO_WARN_STREAM("Controller has already been created!");
    emergencyStop();
    return true;
  }
  try {

    if (!this->create(dt)) {
      this->isCreated_ = false;
      ROCO_WARN_STREAM("Controller could not be created!");
      emergencyStop();
      return true;
    }
    this->isCreated_ = true;
  } catch (std::exception& e) {
    ROCO_WARN_STREAM("Exception caught: " << e.what());
    this->isCreated_ = false;
    emergencyStop();
    return true;
  }
  return true;
}

template<typename Controller_>
bool ControllerRos<Controller_>::initializeController(double dt)
{
  //--- Check if the controller was created.
  if (!this->isCreated()) {
    ROCO_WARN_STREAM("Controller was not created!");
    emergencyStop();
    return false;
  }
  //---

  //--- Reset instead of initialization if the controller has been already initialized.
  if (this->isInitialized()) {
    return resetController(dt);
  }
  //---

  //--- Initialize the controller now.
  try {
    // Update the state.
    updateState(dt, false);
    signal_logger::logger->stopLogger();
    if (!this->initialize(dt)) {
      ROCO_WARN_STREAM("Controller could not be initialized!");
      emergencyStop();
      return false;
    }
    updateCommand(dt, true);
    this->isInitialized_ = true;
  } catch (std::exception& e) {
    ROCO_WARN_STREAM("Exception caught:\n" << e.what());
    this->isInitialized_ = false;
    emergencyStop();
    return false;
  }
  //---

  this->isRunning_ = true;
  signal_logger::logger->startLogger();
  ROCO_INFO_STREAM(
      "Initialized controller " << this->getName() << " successfully!");
  return true;
}

template<typename Controller_>
bool ControllerRos<Controller_>::resetController(double dt)
{
  if (!this->isCreated()) {
    ROCO_WARN_STREAM("Controller has not been created!");
    emergencyStop();
    return false;
  }
  if (!this->isInitialized()) {
    return initializeController(dt);
  }

  try {
    updateState(dt, false);
    signal_logger::logger->stopLogger();
    if (!this->reset(dt)) {
      ROCO_WARN_STREAM("Could not reset controller!");
      emergencyStop();
      return true;
    }
    updateCommand(dt, true);
  } catch (std::exception& e) {
    ROCO_WARN_STREAM("Exception caught: " << e.what());
    emergencyStop();
    return false;
  }

  this->isRunning_ = true;
  signal_logger::logger->startLogger();
  ROCO_INFO_STREAM("Reset controller " << this->getName() << " successfully!");
  return true;
}

template<typename Controller_>
bool ControllerRos<Controller_>::advanceController(double dt)
{
  if (!this->isInitialized()) {
    ROCO_WARN_STREAM("Controller was not initialized!");
    emergencyStop();
    return true;
  }

  try {
    updateState(dt);
    if (!this->advance(dt)) {
      ROCO_WARN_STREAM("Controller::advance() returned false!");
      emergencyStop();
      return true;
    }
    updateCommand(dt, false);
    if (signal_logger::logger) signal_logger::logger->collectLoggerData();
  } catch (std::exception& e) {
    ROCO_WARN_STREAM("Exception caught: " << e.what());
    emergencyStop();
    return true;
  }

  return true;
}

template<typename Controller_>
bool ControllerRos<Controller_>::changeController()
{
  if (!this->isInitialized()) {
    ROCO_WARN_STREAM("Controller is not initialized!");
    emergencyStop();
    return true;
  }
  try {
    if (!this->change()) {
      ROCO_WARN_STREAM("Controller change returned false!");
      emergencyStop();
      return true;
    }
  } catch (std::exception& e) {
    ROCO_WARN_STREAM("Exception caught: " << e.what());
    emergencyStop();
    return true;
  }
  return true;
}

template<typename Controller_>
bool ControllerRos<Controller_>::cleanupController()
{
  if (!this->isCreated()) {
    ROCO_WARN_STREAM("Controller is not created!");
    return false;
  }
  try {
    if (!this->cleanup()) {
      ROCO_WARN_STREAM("Could not clean up the controller!");
      return false;
    }
  } catch (std::exception& e) {
    ROCO_WARN_STREAM("Exception caught: " << e.what());
    return false;
  }
  this->isInitialized_ = false;
  this->isCreated_ = false;
  return true;
}

template<typename Controller_>
bool ControllerRos<Controller_>::updateState(double dt, bool checkState)
{
  time_.setNow();

  if (checkState && isCheckingState_) {
    if (!state_.checkState()) {
      ROCO_ERROR("Bad state!");
      return false;
    }
  }
  return true;
}



template<typename Controller_>
bool ControllerRos<Controller_>::updateCommand(double dt,
                                               bool forceSendingControlModes)
{

  if (isCheckingCommand_) {
    if (!command_.limitCommand()) {
      ROCO_ERROR("The command is invalid!");
    }
  }

  return true;
}

template<typename Controller_>
void ControllerRos<Controller_>::sendEmergencyCommand()
{
  for (auto& command : command_.getActuatorCommands()) {
    command.setMode(robot_model::Command::Mode::MODE_FREEZE);
  }
}

template<typename Controller_>
bool ControllerRos<Controller_>::stopController()
{
  this->isRunning_ = false;
  this->emergencyStop();
  return true;
}
template<typename Controller_>
void ControllerRos<Controller_>::emergencyStop()
{
  ROCO_INFO("ControllerRos::emergencyStop() called!");
  sendEmergencyCommand();
  if (this->getName() != emergencyStopControllerName_) {
    signal_logger::logger->stopLogger();
    signal_logger::logger->saveLoggerData();
    controllerManager_->switchControllerAfterEmergencyStop();
  }
}

template<typename Controller_>
void ControllerRos<Controller_>::setControllerManager(ControllerManager* controllerManager) {
  controllerManager_ = controllerManager;
}

}  // namespace locomotion_controller
