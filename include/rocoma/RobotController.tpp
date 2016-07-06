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

#include <message_logger/message_logger.hpp>

#include <rocoma/RobotController.hpp>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
RobotController<Controller_, State_, Command_>::RobotController(State& state,
                                                            Command& command,
                                                            boost::shared_mutex& mutexState,
                                                            boost::shared_mutex& mutexCommand)  :
                                                            Controller(),
                                                            isRealRobot_(false),
                                                            isCheckingCommand_(true),
                                                            isCheckingState_(true),
                                                            isEmergencyController_(false),
                                                            time_(),
                                                            state_(state),
                                                            mutexState_(mutexState),
                                                            command_(command),
                                                            mutexCommand_(mutexCommand),
                                                            controllerManager_(nullptr)
                                                            {
  // always check state/command on the real robot
  if (isRealRobot_) {
    isCheckingCommand_ = true;
    isCheckingState_ = true;
  }

  // emergency controller does not check state
  if (isEmergencyController_) {
    isCheckingState_ = false;
    isCheckingCommand_ = true;
  }

                                                            }

template<typename Controller_, typename State_, typename Command_>
RobotController<Controller_, State_, Command_>::~RobotController()
{
  if (!cleanupController()) {
    MELO_ERROR("Could not cleanup controller!");
  }
}

template<typename Controller_, typename State_, typename Command_>
const roco::time::Time& RobotController<Controller_,State_, Command_>::getTime() const
{
  return static_cast<const roco::time::Time&>(time_);
}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_,State_, Command_>::setTime(const roco::time::Time& time)
{
  time_ = time;
}

template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_, State_, Command_>::isCheckingCommand() const
{
  return isCheckingCommand_;
}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_, State_, Command_>::setIsCheckingCommand(bool isChecking)
{
  MELO_WARN("Checking the command was deactivated!");
  isCheckingCommand_ = isChecking;
}

template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_,State_, Command_>::isCheckingState() const
{
  return isCheckingState_;
}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_,State_, Command_>::setIsCheckingState(bool isChecking)
{
  MELO_WARN("Checking the robot state was deactivated!");
  isCheckingState_ = isChecking;
}

// ------------------------------------------------------------------------------------------------------------------------
//! TODO use worker manager from philipp to implement this
template<typename Controller_, typename State_, typename Command_>
roco::WorkerHandle RobotController<Controller_, State_, Command_>::addWorker(const roco::WorkerOptions&  options) {
  workers_.emplace(options.name_, WorkerWrapper());
  auto& wrapper = workers_[options.name_];
  wrapper.options_ = options;

  nodewrap::WorkerOptions workerOptions;
  workerOptions.autostart = options.autostart_;
  workerOptions.frequency = options.frequency_;
  workerOptions.priority = options.priority_;
  workerOptions.synchronous = options.synchronous_;
  workerOptions.callback = boost::bind(&WorkerWrapper::workerCallback, wrapper, _1);

  wrapper.worker_ = controllerManager_->getLocomotionController()->addWrappedWorker(options.name_, workerOptions);

  roco::WorkerHandle workerHandle;
  workerHandle.name_ = options.name_;

  return workerHandle;
}

template<typename Controller_, typename State_, typename Command_>
roco::WorkerHandle RobotController<Controller_, State_, Command_>::addWorker(roco::Worker& worker) {

  auto& options  = worker.options_;
  workers_.emplace(options.name_, WorkerWrapper());
  auto& wrapper = workers_[options.name_];

  wrapper.options_ = options;

  nodewrap::WorkerOptions workerOptions;
  workerOptions.autostart =  options.autostart_;
  workerOptions.frequency =  options.frequency_;
  workerOptions.priority =  options.priority_;
  workerOptions.synchronous =  options.synchronous_;
  workerOptions.callback = boost::bind(&WorkerWrapper::workerCallback, wrapper, _1);
  wrapper.worker_ = controllerManager_->getLocomotionController()->addWrappedWorker(options.name_, workerOptions);

  worker.workerStartCallback_ = boost::bind(&RobotController<Controller_,State_, Command_>::startWorker, this, _1);
  worker.workerCancelCallback_ = boost::bind(&RobotController<Controller_,State_, Command_>::cancelWorker, this, _1, _2 );

  worker.handle_.name_ = options.name_;
  return worker.handle_;
}

template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_, State_, Command_>::startWorker(const roco::WorkerHandle& workerHandle) {
  MELO_INFO_STREAM("ControllerRos::startWorker: start " << workerHandle.name_);
  workers_[workerHandle.name_].worker_.start();
  MELO_INFO_STREAM("ControllerRos::startWorker started " << workerHandle.name_);
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_, State_, Command_>::cancelWorker(const roco::WorkerHandle& workerHandle, bool block) {
  MELO_INFO_STREAM("ControllerRos::cancelWorker: cancel  " << workerHandle.name_);
  workers_[workerHandle.name_].worker_.cancel(block);
  return true;
}
// ------------------------------------------------------------------------------------------------------------------------

template<typename Controller_, typename State_, typename Command_>
const typename RobotController<Controller_, State_, Command_>::State& RobotController<Controller_, State_, Command_>::getState() const
{
  return state_;
}

template<typename Controller_, typename State_, typename Command_>
boost::shared_mutex& RobotController<Controller_, State_, Command_>::getStateMutex()
{
  return mutexState_;
}

template<typename Controller_, typename State_, typename Command_>
const typename RobotController<Controller_, State_, Command_>::Command& RobotController<Controller_, State_, Command_>::getCommand() const
{
  return command_;
}

template<typename Controller_, typename State_, typename Command_>
typename RobotController<Controller_, State_, Command_>::Command& RobotController<Controller_, State_, Command_>::getCommand()
{
  return command_;
}

template<typename Controller_, typename State_, typename Command_>
boost::shared_mutex& RobotController<Controller_, State_, Command_>::getCommandMutex()
{
  return mutexCommand_;
}

// ------------------------------------------------------------------------------------------------------------------------
//! TODO properly implement these methods
template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_, State_, Command_>::createController(double dt)
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
bool RobotController<Controller_, State_, Command_>::initializeController(double dt)
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
bool RobotController<Controller_,State_, Command_>::resetController(double dt)
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
bool RobotController<Controller_,State_, Command_>::advanceController(double dt)
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
bool RobotController<Controller_,State_, Command_>::changeController()
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
bool RobotController<Controller_,State_, Command_>::cleanupController()
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
bool Controller<Controller_,State_, Command_>::stopController()
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
bool RobotController<Controller_,State_, Command_>::preStopController() {

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
void RobotController<Controller_,State_, Command_>::swapOut() {
  ROS_INFO("Calling swap out for controller %s",this->getName().c_str());
  cancelWorkers();
  this->isRunning_ = false;
  try {
    if(!this->stop()) {
      MELO_WARN("Could not stop controller %s while swapping out!", this->getName().c_str());
    }
  }
  catch (std::exception& e) {
    MELO_WARN_STREAM("Could not stop controller " << this->getName() << " while swapping out! Exception caught: " << e.what());
  }
}
// ------------------------------------------------------------------------------------------------------------------------


template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_, State_, Command_>::setControllerManager(ControllerManager* controllerManager) {
  controllerManager_ = controllerManager;
}





template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_,State_, Command_>::isRealRobot() const
{
  return isRealRobot_;
}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_,State_, Command_>::setIsRealRobot(bool isRealRobot)
{
  MELO_WARN("isRealRobot was set to %d for controller %s ", isRealRobot, this->getName());
  isRealRobot_ = isRealRobot;
}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_,State_, Command_>::emergencyStop()
{
  MELO_INFO("Controller::emergencyStop() called!");
  sendEmergencyCommand();

  if (this->getName() != emergencyStopControllerName_) {
#ifdef LC_ENABLE_LOGGER
    {
      signal_logger::logger->stopLogger();
      signal_logger::logger->saveLoggerData();
    }
#endif
    // todo: check if stop is needed here
    //    if (this->isRunning_){
    //      try {
    //        if(!this->stop()) {
    //          ROCO_WARN("Could not stop controller %s after emergency stop!", this->getName().c_str());
    //        }
    //      }
    //      catch (std::exception& e) {
    //        ROCO_WARN_STREAM("Could not stop controller " << this->getName() << " after emergency stop! Exception caught: " << e.what());
    //      }
    //    }

    controllerManager_->switchControllerAfterEmergencyStop();
  }
  cancelWorkers();
  controllerManager_->notifyEmergencyState();

}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_,State_, Command_>::startWorkers() {
  ROCO_INFO_STREAM("[" << this->getName() << "] Starting workers.");
#ifdef LC_ENABLE_LOGGER
  signalLoggerWorker_.start();
#endif
}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_,State_, Command_>::cancelWorkers() {
  ROCO_INFO_STREAM("[" << this->getName() << "] Canceling workers.");
#ifdef LC_ENABLE_LOGGER
  signalLoggerWorker_.cancel(true);
#endif
}

template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_,State_, Command_>::updateState(double dt, bool checkState)
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
bool RobotController<Controller_,State_, Command_>::updateCommand(double dt, bool forceSendingControlModes) {

  if (isCheckingCommand_) {
    boost::unique_lock<boost::shared_mutex> lock(getCommandMutex());
    if (!command_.limitCommand()) {
      ROCO_ERROR("The command is invalid!");
    }
  }

  return true;
}

template<typename Controller_, typename State_, typename Command_>
void RobotController<Controller_,State_, Command_>::sendEmergencyCommand()
{
  boost::unique_lock<boost::shared_mutex> lock(getCommandMutex());
  for (auto& command : command_.getActuatorCommands()) {
    command.setMode(quadruped_model::Command::Mode::MODE_FREEZE);
  }
}

template<typename Controller_, typename State_, typename Command_>
bool RobotController<Controller_,State_, Command_>::signalLoggerWorker(const nodewrap::WorkerEvent& event) {
  //#ifdef LC_ENABLE_LOGGER
  //  signal_logger::logger->collectLoggerData();
  //#endif
  return true;
}

}  // namespace rocoma
