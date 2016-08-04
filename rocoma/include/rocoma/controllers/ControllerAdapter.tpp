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
 * @author   Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Dec, 2014
 * @brief
 */

// Message logger
#include <message_logger/message_logger.hpp>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::createController(double dt)
{
  // Check if controller was already created
  if (this->isCreated()) {
    MELO_WARN_STREAM("[ControllerAdapter]: Controller has already been created!");
    return true;
  }

  try {

    // create controller
    if (!this->create(dt)) {
      this->isCreated_ = false;
      MELO_WARN_STREAM("[ControllerAdapter]: Controller could not be created!");
      return false;
    }

    // set flag
    this->isCreated_ = true;

  } catch (std::exception& e) {
    MELO_WARN_STREAM("[ControllerAdapter]: Exception caught: " << e.what());
    this->isCreated_ = false;
    return false;
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::initializeController(double dt)
{
  // Check if controller was created
  if (!this->isCreated()) {
    MELO_WARN_STREAM("[ControllerAdapter]: Controller was not created!");
    return false;
  }

  // Reset instead of initialization if the controller has been already initialized
  if (this->isInitialized()) {
    return resetController(dt);
  }

  try {
    // Update the state
    updateState(dt, false);

    // Initialize the controller
    if (!this->initialize(dt)) {
      MELO_WARN_STREAM("[ControllerAdapter]: Controller could not be initialized!");
      return false;
    }

    // Update the command
    updateCommand(dt);

    // Set init flag
    this->isInitialized_ = true;

  }
  catch (std::exception& e) {
    MELO_WARN_STREAM("[ControllerAdapter]: Exception caught:\n" << e.what());
    this->isInitialized_ = false;
    return false;
  }

  // Set running flag
  this->isRunning_ = true;
  MELO_INFO_STREAM("[ControllerAdapter]: Initialized controller " << this->getName() << " successfully!");

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::advanceController(double dt)
{
  if (!this->isInitialized()) {
    MELO_WARN_STREAM("Controller was not initialized!");
    return false;
  }

  try {
    updateState(dt);
    if (!this->advance(dt)) {
      MELO_WARN_STREAM("Controller::advance() returned false!");
      stopController();
      return true;
    }
    updateCommand(dt);

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    stopController();
    return true;
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::resetController(double dt)
{
  // Check if controller was created
  if (!this->isCreated()) {
    MELO_WARN_STREAM("[ControllerAdapter]: Controller was not created!");
    return false;
  }

  // Initialize instead of reset if the controller has not been already initialized
  if (!this->isInitialized()) {
    return initializeController(dt);
  }

  try {
    // Update the state
    updateState(dt, false);

    // Initialize the controller
    if (!this->reset(dt)) {
      MELO_WARN_STREAM("[ControllerAdapter]: Controller could not be reset!");
      return false;
    }

    // Update the command
    updateCommand(dt);
  }
  catch (std::exception& e) {
    MELO_WARN_STREAM("[ControllerAdapter]: Exception caught:\n" << e.what());
    return false;
  }

  // Set running flag
  this->isRunning_ = true;
  MELO_INFO_STREAM("[ControllerAdapter]: Reset controller " << this->getName() << " successfully!");

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::cleanupController()
{
  MELO_INFO_STREAM("[ControllerAdapter]: Cleaning controller " << this->getName() << " up.");

  // Check if controller was created
  if (!this->isCreated()) {
    MELO_WARN_STREAM("[ControllerAdapter]: Controller is not created!");
    return false;
  }

  // Cleanup the controller
  try {
    if (!this->cleanup()) {
      MELO_WARN_STREAM("[ControllerAdapter]: Could not clean up the controller!");
      return false;
    }

  } catch (std::exception& e) {
    MELO_WARN_STREAM("[ControllerAdapter]: Exception caught: " << e.what());
    return false;
  }

  // Reset flags
  this->isInitialized_ = false;
  this->isCreated_ = false;
  this->isRunning_ = false;

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::stopController()
{
  // set flag
  this->isRunning_ = false;

  sleep(5);         //make the programme waiting for 5 secondes
  // stop controller
  try {
    if(!this->stop()) {
      MELO_WARN("[ControllerAdapter]: Could not stop controller %s!", this->getName().c_str());
      return false;
    }
  } catch (std::exception& e) {
    MELO_WARN_STREAM("[ControllerAdapter]: Could not stop controller " << this->getName() << "! Exception caught: " << e.what());
    return false;
  }

  this->isStopping_ = false;
  return true;
}


template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::preStopController()
{
  this->isStopping_ = true;

  try {
    if(!this->preStop()) {
      MELO_WARN("[ControllerAdapter]: Could not prepare to stop controller %s!", this->getName().c_str());
      return false;
    }
  } catch (std::exception& e) {
    MELO_WARN_STREAM("[ControllerAdapter]: Could not prepare to stop controller " << this->getName() << "! Exception caught: " << e.what());
    return false;
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateState(double dt, bool checkState)
{
  this->time_.setNow();

  if (checkState && this->isCheckingState_) {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    if (!this->getState().checkState()) {
      MELO_ERROR("[ControllerAdapter]: Bad state!");
      return false;
    }
  }
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateCommand(double dt)
{
  if (this->isCheckingCommand_) {
    boost::unique_lock<boost::shared_mutex> lock(this->getCommandMutex());
    if (!this->getCommand().limitCommand()) {
      MELO_ERROR("[ControllerAdapter]: The command is invalid!");
      return false;
    }
  }

  return true;
}

}  // namespace rocoma
