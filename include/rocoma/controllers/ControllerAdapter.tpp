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
ControllerAdapter<Controller_, State_, Command_>::ControllerAdapter()  :
Controller()
{


}

template<typename Controller_, typename State_, typename Command_>
ControllerAdapter<Controller_, State_, Command_>::~ControllerAdapter()
{
  if (!cleanupController()) {
    MELO_ERROR("Could not cleanup controller!");
  }
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::createController(double dt)
{
  if (!this->isCreated()) {
    try {
      // create controller if not already created (handle exeptions)
      this->isCreated_ = this->create(dt);
    } catch (std::exception& e) {
      this->isCreated_ = false;
      MELO_WARN_STREAM("Exception caught during controller creation: " << e.what());
      emergencyStop();
      return true;
    }

    // start logger if controller was created successfully, print warning otherwise
    if(this->isCreated()) {
      //      #ifdef LC_ENABLE_LOGGER
      //          //--- start signal logging in a worker thread
      //          nodewrap::WorkerOptions signalLoggerWorkerOptions;
      //          signalLoggerWorkerOptions.autostart = false;
      //          signalLoggerWorkerOptions.frequency = signal_logger::logger->getSamplingFrequency();
      //          signalLoggerWorkerOptions.callback = boost::bind(
      //              &Controller<Controller_,State_, Command_>::signalLoggerWorker, this, _1);
      //
      //          const std::string& signalLoggerWorkerName = this->getName()
      //                              + "_signal_logger_worker";
      //          signalLoggerWorker_ = controllerManager_->getLocomotionController()
      //                              ->addWrappedWorker(signalLoggerWorkerName, signalLoggerWorkerOptions);
      //      #endif
    }
    else {
      MELO_WARN("Controller could not be created!");
      emergencyStop();
      return true;
    }
  }
  else {
    MELO_WARN("Controller has already been created!");
    emergencyStop();
    return true;
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::initializeController(double dt)
{
  // Check if the controller was created.
  if (!this->isCreated()) {
    MELO_WARN("Controller was not created!");
    emergencyStop();
    return false;
  }

  // Reset instead of initialization if the controller has been already initialized.
  if (this->isInitialized()) {
    return resetController(dt);
  }

  // Initialize the controller now.
  try {
    // Update the state.
    updateState(dt, false);

    // Stop logger
#ifdef LC_ENABLE_LOGGER
    signal_logger::logger->stopLogger();
#endif

    // initialize controller
    if (!this->initialize(dt)) {
      MELO_WARN("Controller could not be initialized!");
      emergencyStop();
      return false;
    }

    // Update the command
    updateCommand(dt, true);

    this->isInitialized_ = true;

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught:\n" << e.what());
    this->isInitialized_ = false;
    emergencyStop();
    return false;
  }
  //---

  this->isRunning_ = true;
#ifdef LC_ENABLE_LOGGER
  signal_logger::logger->startLogger();
#endif
  startWorkers();
  MELO_INFO_STREAM(
      "Initialized controller " << this->getName() << " successfully!");
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::resetController(double dt)
{
  if (!this->isCreated()) {
    MELO_WARN("Controller has not been created!");
    emergencyStop();
    return false;
  }
  if (!this->isInitialized()) {
    return initializeController(dt);
  }

  try {
    updateState(dt, false);
#ifdef LC_ENABLE_LOGGER
    signal_logger::logger->stopLogger();
#endif
    if (!this->reset(dt)) {
      MELO_WARN("Could not reset controller!");
      emergencyStop();
      return true;
    }
    updateCommand(dt, true);
  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    emergencyStop();
    return false;
  }

  this->isRunning_ = true;
#ifdef LC_ENABLE_LOGGER
  signal_logger::logger->startLogger();
#endif
  startWorkers();
  MELO_INFO_STREAM("Reset controller " << this->getName() << " successfully!");
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::advanceController(double dt)
{
  if (!this->isInitialized()) {
    MELO_WARN("Controller was not initialized!");
    emergencyStop();
    return true;
  }

  try {
    updateState(dt);
    if (!this->advance(dt)) {
      MELO_WARN("Controller::advance() returned false!");
      stopController();
      return true;
    }
    updateCommand(dt, false);
#ifdef LC_ENABLE_LOGGER

    signal_logger::logger->collectLoggerData();
#endif

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    stopController();
    return true;
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::changeController()
{
  if (!this->isInitialized()) {
    MELO_WARN("Controller is not initialized!");
    emergencyStop();
    return true;
  }
  try {
    if (!this->change()) {
      MELO_WARN("Controller change returned false!");
      emergencyStop();
      return true;
    }
  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    emergencyStop();
    return true;
  }
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::cleanupController()
{
  MELO_INFO_STREAM("Cleaning controller " << this->getName() << " up.");

  if (!this->isCreated()) {
    MELO_WARN("Controller is not created!");
    return false;
  }
  try {
    if (!this->cleanup()) {
      MELO_WARN("Could not clean up the controller!");
      return false;
    }

    cancelWorkers();

  } catch (std::exception& e) {
    MELO_WARN_STREAM("Exception caught: " << e.what());
    return false;
  }
  this->isInitialized_ = false;
  this->isCreated_ = false;
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::stopController()
{
  cancelWorkers();

  this->isRunning_ = false;
  try {
    if(!this->stop()) {
      MELO_WARN("Could not stop controller %s!", this->getName().c_str());
    }
  }
  catch (std::exception& e) {
    MELO_WARN_STREAM("Could not stop controller " << this->getName() << "! Exception caught: " << e.what());
  }
  this->emergencyStop();

  return true;
}


template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::preStopController() {

  try {
    if(!this->preStop()) {
      MELO_WARN("Could not prepare to stop controller %s!", this->getName().c_str());
      return false;
    }
  }
  catch (std::exception& e) {
    MELO_WARN_STREAM("Could not prepare to stop controller " << this->getName() << "! Exception caught: " << e.what());
    this->emergencyStop();
    return false;
  }
  return true;
}

template<typename Controller_, typename State_, typename Command_>
void ControllerAdapter<Controller_,State_, Command_>::setIsRealRobot(bool isRealRobot)
{
  MELO_WARN("isRealRobot was set to %d for controller %s ", isRealRobot, this->getName());
  isRealRobot_ = isRealRobot;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateState(double dt, bool checkState)
{
  time_.setNow();

  if (checkState && isCheckingState_) {
    boost::shared_lock<boost::shared_mutex> lock(getStateMutex());
    if (!state_.checkState()) {
      ROCO_ERROR("Bad state!");
      return false;
    }
  }
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateCommand(double dt, bool forceSendingControlModes) {

  if (isCheckingCommand_) {
    boost::unique_lock<boost::shared_mutex> lock(getCommandMutex());
    if (!command_.limitCommand()) {
      ROCO_ERROR("The command is invalid!");
    }
  }

  return true;
}

}  // namespace rocoma
