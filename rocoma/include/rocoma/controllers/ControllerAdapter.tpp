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
 * @file     ControllerAdapter.tpp
 * @author   Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

// Message logger
#include <message_logger/message_logger.hpp>

// Signal logger
#include <signal_logger/signal_logger.hpp>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::createController(double dt)
{
  if (this->isCreated()) {
    MELO_WARN_STREAM("Controller has already been created!");
    return true;
  }

  try {
    // Create controller
    if (!this->create(dt)) {
      this->isCreated_ = false;
      MELO_WARN_STREAM("Controller could not be created!");
      return false;
    }

    // Set flag
    this->isCreated_ = true;

  } catch (std::exception& e) {
    //! return false (let manager handle this)
    MELO_WARN_STREAM("Exception caught: " << e.what());
    this->isCreated_ = false;
    return false;
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::initializeController(double dt)
{
  // Check if the controller was created.
  if (!this->isCreated()) {
    MELO_WARN_STREAM("Controller was not created!");
    return false;
  }

  // Reset instead of initialization if the controller has been already initialized.
  if (this->isInitialized()) {
    return resetController(dt);
  }

  // Initialize the controller now.
  try {
    // Update the state.
    if(!this->updateState(dt, false)) {
   	  return false;
   	}
    signal_logger::logger->stopLogger();

    // Initialize controller
    if (!this->initialize(dt)) {
      MELO_WARN_STREAM("Controller could not be initialized!");
      return false;
    }

    // Update command
    if(!this->updateCommand(dt)) {
      return false;
    }

    this->isInitialized_ = true;

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught:\n" << e.what());
    this->isInitialized_ = false;
    return false;
  }

  // Start logging
  this->isRunning_ = true;
  signal_logger::logger->startLogger();
  MELO_INFO_STREAM( "Initialized controller " << this->getControllerName() << " successfully!");

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::advanceController(double dt)
{
  // Check if controller is initialized
  if (!this->isInitialized()) {
    MELO_WARN_STREAM("Controller was not initialized!");
    return false;
  }

  try {
    // Advance controller
    if(!this->updateState(dt)) {
      return false;
    }

    if (!this->advance(dt)) {
      MELO_WARN_STREAM("Controller::advance() returned false!");
      return false;
    }

    // Update commands
    if(!this->updateCommand(dt)) {
      return false;
    }

    // Collect logger data
    signal_logger::logger->collectLoggerData();

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    return false;
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::resetController(double dt)
{
  // Check if controller was created
  if (!this->isCreated()) {
    MELO_WARN_STREAM("Controller has not been created!");
    return false;
  }

  // Initialize if first call
  if (!this->isInitialized()) {
    return initializeController(dt);
  }

  try {
    // Update state
    if(!updateState(dt, false) ) {
      return false;
    }
    signal_logger::logger->stopLogger();

    // Reset controller
    if (!this->reset(dt)) {
      MELO_WARN_STREAM("Could not reset controller!");
      return false;
    }

    // Update command
    if(!updateCommand(dt)) {
      return false;
    }

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    return false;
  }

  // Start logging
  this->isRunning_ = true;
  signal_logger::logger->startLogger();
  MELO_INFO_STREAM("Reset controller " << this->getControllerName() << " successfully!");

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::cleanupController()
{
  MELO_INFO_STREAM("Cleaning controller " << this->getControllerName() << " up.");

  // Check if controller was created
  if (!this->isCreated()) {
    MELO_WARN_STREAM("Controller is not created!");
    return false;
  }

  // Cleanup controllers
  try {
    if (!this->cleanup()) {
      MELO_WARN_STREAM("Could not clean up the controller!");
      return false;
    }

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    return false;
  }

  // Set flags
  this->isInitialized_ = false;
  this->isCreated_ = false;

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::stopController()
{
  // Set flag
  this->isRunning_ = false;

  // Collect logger data
  signal_logger::logger->stopLogger();
  signal_logger::logger->saveLoggerData();

  // Stop controller
  try {
    if(!this->stop()) {
      MELO_WARN("Could not stop controller %s!", this->getName().c_str());
      return false;
    }
  }
  catch (std::exception& e) {
    MELO_WARN_STREAM("Could not stop controller " << this->getName() << "! Exception caught: " << e.what());
    return false;
  }

  return true;
}


template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::preStopController()
{
  try {
    if(!this->preStop()) {
      MELO_WARN("Could not prepare to stop controller %s!", this->getName().c_str());
      return false;
    }
  }
  catch (std::exception& e) {
    MELO_WARN_STREAM("Could not prepare to stop controller " << this->getName() << "! Exception caught: " << e.what());
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
      MELO_ERROR("Bad state!");
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
      MELO_ERROR("The command is invalid!");
      return false;
    }
  }

  return true;
}

}  // namespace rocoma
