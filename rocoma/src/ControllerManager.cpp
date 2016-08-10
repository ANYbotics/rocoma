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
 * @author  Christian Gehring, Gabriel Hottiger
 * @date    Oct, 2014
 */
#include "rocoma/ControllerManager.hpp"

#include "message_logger/message_logger.hpp"
#include <limits>

namespace rocoma {

ControllerManager::ControllerManager() :
                                                //    updating_(false),
                                                //    timerStart_(),
                                                //    timerStop_(),
                                                //    minimalRealtimeFactor_(2.0),
                                                    timeStep_(0.1),
                                                    isRealRobot_(false),
                                                    activeControllerState_(State::FAILURE),
                                                    workerManager_(),
                                                    controllers_(),
                                                    emergencyControllers_(),
                                                    controllerPairs_(),
                                                    activeControllerPair_(nullptr, nullptr),
                                                    failproofController_(nullptr),
                                                    controllerMutex_(),
                                                    emergencyControllerMutex_(),
                                                    emergencyStopMutex_(),
                                                    updateControllerMutex_(),
                                                    switchControllerMutex_(),
                                                    workerManagerMutex_(),
                                                    activeControllerMutex_()
{
  //  any_worker::WorkerOptions checkTimingWorkerOptions;
  //  checkTimingWorkerOptions.name_ = "check_timing";
  //  checkTimingWorkerOptions.timeStep_ = 0;
  //  checkTimingWorkerOptions.callback_ = std::bind(&ControllerManager::checkTimingWorker, this, std::placeholders::_1);
  //  checkTimingWorkerOptions.defaultPriority_ = 0;
  //  checkTimingWorkerOptions.destructWhenDone_ = false;
  //  workerManager_.addWorker(checkTimingWorkerOptions, true);
}


ControllerManager::~ControllerManager()
{
}

bool ControllerManager::addControllerPair(std::unique_ptr<roco::ControllerAdapterInterface> controller,
                                          std::unique_ptr<roco::EmergencyControllerAdapterInterface> emergencyController) {

  // Check for invalid controller
  if(controller == nullptr) {
    MELO_ERROR_STREAM("Could not add controller pair. Controller is nullptr.");
    return false;
  }

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();
  const std::string emgcyControllerName = (emergencyController == nullptr)?"FailproofController": emergencyController->getControllerName();

  MELO_INFO_STREAM("Adding controller pair ctrl: " << controllerName << " / emgcy ctrl: " << emgcyControllerName << " ... ");

  // check if controller already exists
  if(controllers_.find(controllerName) != controllers_.end()) {
    MELO_WARN_STREAM("... Could not add controller " << controllerName << ". A controller with the same name already exists.");
    return false;
  }

  //--- Add controller
  MELO_INFO_STREAM(" Adding controller " << controllerName << " ... ");

  // create controller
  if (!controller->createController(timeStep_)) {
    MELO_ERROR_STREAM("... Could not create controller " << controllerName << "!");
    return false;
  }

  // insert controller (move ownership to controller / controller is set to nullptr)
  controllers_.insert( std::pair<std::string, ControllerPtr >(controllerName, ControllerPtr( std::move(controller) ) ) );
  MELO_INFO_STREAM("... successfully added controller " << controllerName << "!");

  //--- Add emergency controller
  MELO_INFO_STREAM(" Adding emergency controller " << emgcyControllerName << " ... ");

  if(emergencyController == nullptr)
  {
    controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName, ControllerSetPtr(controllers_.at(controllerName).get(), nullptr ) ) );
    MELO_INFO_STREAM("... sucessfully added controller pair ctrl: " << controllerName << " / emgcy ctrl: " << emgcyControllerName << " ... ");
    return true;
  }

  // create emergency controller
  if (!emergencyController->createController(timeStep_)) {
    MELO_ERROR_STREAM("... Could not create emergency controller " << emgcyControllerName << "! Use failproof controller on emergency stop!");
    controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName, ControllerSetPtr(controllers_.at(controllerName).get(), nullptr) ) );
    return false;
  }

  // check if emergency controller already exists
  if(emergencyControllers_.find(emgcyControllerName) != emergencyControllers_.end()) {
    MELO_WARN_STREAM("... Could not add emergency controller " << emgcyControllerName << ". An emergency controller with the same name already exists.");
  }
  else {
    // insert emergency controller (move ownership to controller / controller is set to nullptr)
    emergencyControllers_.insert( std::pair<std::string, EmgcyControllerPtr>(emgcyControllerName, EmgcyControllerPtr( std::move(emergencyController) ) ) );
    MELO_INFO_STREAM("... successfully added emergency controller " << emgcyControllerName << "!");
  }

  // Add controller pair and set active
  controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName,
                                                                       ControllerSetPtr(controllers_.at(controllerName).get(), emergencyControllers_.at(emgcyControllerName).get() ) ) );
  MELO_INFO_STREAM("... sucessfully added controller pair ctrl: " << controllerName << " / emgcy ctrl: " << emgcyControllerName << " ... ");

  return true;
}

bool ControllerManager::setFailproofController(std::unique_ptr<roco::FailproofControllerAdapterInterface> controller)  {

  // controller name (controller is moved within this function, therefore not safe to access everywhere)
  std::string controllerName = controller->getControllerName();
  MELO_INFO("Adding failproof controller %s ...", controllerName.c_str());

  // create controller
  if (!controller->createController(timeStep_)) {
    MELO_ERROR("Could not create failproof controller %s!", controllerName.c_str());
    return false;
  }

  // move controller
  failproofController_ = std::unique_ptr<roco::FailproofControllerAdapterInterface>( std::move(controller) );
  MELO_INFO("... finished adding failproof controller %s.", controllerName.c_str());

  return true;
}

bool ControllerManager::updateController() {
  // Call to update Controller is sequential
  std::unique_lock<std::mutex> lockUpdate(updateControllerMutex_);

  //  // start update
  //  updating_ = true;
  //  timerStart_.notify_one();

  // Controller is running
  if(activeControllerState_ == State::OK)
  {
    // Advance "normal" controller -> if advance return false treat as emergency stop
    std::unique_lock<std::mutex> lockController(controllerMutex_);
    if(!activeControllerPair_.controller_->advanceController(timeStep_))
    {
      lockController.unlock();
      emergencyStop();
    }
  }

  // Controller is in emergency stop
  if(activeControllerState_ == State::EMERGENCY)
  {
    std::unique_lock<std::mutex> lockEmergencyController(emergencyControllerMutex_);
    if(!activeControllerPair_.emgcyController_->advanceController(timeStep_))
    {
      lockEmergencyController.unlock();
      emergencyStop();
    }
  }

  // Failproof controller is active
  if(activeControllerState_ == State::FAILURE)
  {
    // returns void -> can never fail!
    failproofController_->advanceController(timeStep_);
  }

  //  // stop update
  //  updating_ = false;
  //  timerStop_.notify_one();

  return true;
}

bool ControllerManager::emergencyStop() {

  // Cannot call emergency stop twice simultaniously
  std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);

  // Clean workers
  {
    std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
    workerManager_.cleanDestructibleWorkers();
  }

  // Check if controller is in failproof state already
  if(activeControllerState_ == State::FAILURE)
  {
    MELO_WARN("Can not emergency stop! Failproof controller is already running!");
    return false;
  }

  // Stop worker options
  any_worker::WorkerOptions stopWorkerOptions;
  stopWorkerOptions.timeStep_ = std::numeric_limits<double>::infinity();
  stopWorkerOptions.defaultPriority_ = 0;
  stopWorkerOptions.destructWhenDone_ = true;

  // If state ok and emergency controller registered -> switch to emergency controller
  if(activeControllerState_ == State::OK &&
     activeControllerPair_.emgcyController_ != nullptr &&
     !activeControllerPair_.emgcyController_->isBeingStopped() )
  {
    // Init emergency controller fast
    {
      std::unique_lock<std::mutex> lockEmergencyController(emergencyControllerMutex_);
      activeControllerPair_.emgcyController_->initializeControllerFast(timeStep_);
    }

    // Switch to emergency state
    activeControllerState_ = State::EMERGENCY;

    // stop controller in a different thread
    stopWorkerOptions.name_ = "stop_controller_" + activeControllerPair_.controller_->getControllerName();
    stopWorkerOptions.callback_ = std::bind(&ControllerManager::emergencyStopControllerWorker, this, std::placeholders::_1,
                                            activeControllerPair_.controller_, EmergencyStopType::EMERGENCY);
  }
  else {
    // Switch to failure state
    activeControllerState_ = State::FAILURE;

    // stop emergency controller in a different thread
    stopWorkerOptions.name_ = "stop_controller_" + activeControllerPair_.emgcyController_->getControllerName();
    stopWorkerOptions.callback_ = std::bind(&ControllerManager::emergencyStopControllerWorker, this, std::placeholders::_1,
                                            activeControllerPair_.emgcyController_, EmergencyStopType::FAILPROOF);

  }

  // Stop old controller in a separate thread
  {
    std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
    workerManager_.addWorker(stopWorkerOptions, true);
  }

  return true;
}

bool ControllerManager::emergencyStopControllerWorker(const any_worker::WorkerEvent& e,
                                                      roco::ControllerAdapterInterface * controller,
                                                      EmergencyStopType emgcyStopType)
{
  bool success = true;

  {
    // Stop controller and block -> switch controller can not happen while controller is stopped
    std::unique_lock<std::mutex> lockController(controllerMutex_);
    controller->setIsBeingStopped(true);
    success = controller->preStopController();
    success = controller->stopController() && success;
    controller->setIsBeingStopped(false);
  }

  // notify emergency stop
  notifyEmergencyStop(emgcyStopType);

  return success;
}

ControllerManager::SwitchResponse ControllerManager::switchController(const std::string & controllerName) {

  // Allow only sequential calls to switch controller
  std::unique_lock<std::mutex> lockSwitchController(switchControllerMutex_);

  // Make sure were not in emergency stop procedure
  {
    std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);
  }

  // Check if controller is already active
  {
    std::unique_lock<std::mutex> lockActiveController(activeControllerMutex_);
    if (activeControllerState_ == State::OK && controllerName == activeControllerPair_.controller_->getControllerName()) {
      MELO_INFO("Controller %s is already running!", controllerName.c_str());
      return SwitchResponse::RUNNING;
    }
  }

  // Find controller
  auto controllerPair = controllerPairs_.find(controllerName);
  if(controllerPair != controllerPairs_.end()) {
    // Switch controller worker
    any_worker::WorkerOptions switchControllerWorkerOptions;
    switchControllerWorkerOptions.timeStep_ = std::numeric_limits<double>::infinity();
    switchControllerWorkerOptions.defaultPriority_ = 0;
    switchControllerWorkerOptions.destructWhenDone_ = true;

    // Define callback name and controllers to be switched
    switch(activeControllerState_) {
      case State::OK:
      {
        switchControllerWorkerOptions.name_ = "switch_from_" + activeControllerPair_.controllerName_ + "_to_" + controllerPair->second.controllerName_;
        switchControllerWorkerOptions.callback_ = std::bind(&ControllerManager::switchControllerWorker, this,
                                                            std::placeholders::_1, activeControllerPair_.controller_, controllerPair->second.controller_);
        break;
      }
      case State::EMERGENCY:
      {
        switchControllerWorkerOptions.name_ = "switch_from_" + activeControllerPair_.emgcyControllerName_ + "_to_" + controllerPair->second.controllerName_;
        switchControllerWorkerOptions.callback_ = std::bind(&ControllerManager::switchControllerWorker, this,
                                                            std::placeholders::_1, activeControllerPair_.emgcyController_, controllerPair->second.controller_);
        break;
      }
      case State::FAILURE:
      {
        switchControllerWorkerOptions.name_ = "switch_from_failproof_to_" + controllerPair->second.controller_->getControllerName();
        switchControllerWorkerOptions.callback_ = std::bind(&ControllerManager::switchControllerWorker, this,
                                                            std::placeholders::_1, nullptr, controllerPair->second.controller_);
        break;
      }
    }

    {
      // Add worker to switch to new controller
      std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
      workerManager_.cleanDestructibleWorkers();
      workerManager_.addWorker(switchControllerWorkerOptions, true);
    }

    return SwitchResponse::SWITCHING;
  }
  else {
    // controller is not part of controller map
    MELO_INFO("Controller %s not found!", controllerName.c_str());
    return SwitchResponse::NOTFOUND;
  }

  return SwitchResponse::ERROR;
}

bool ControllerManager::switchControllerWorker(const any_worker::WorkerEvent& e,
                                               roco::ControllerAdapterInterface * oldController,
                                               roco::ControllerAdapterInterface * newController) {
  /** NOTE:
   * 1. The active controller is not blocked -> by definition there can be no data races between advance and preStop
   * 2. Set this oldController to beeing stopped to prevent a switch to it
   */
  // shutdown communication for active controller
  if(oldController != nullptr) {
    oldController->setIsBeingStopped(true);
    oldController->preStopController();
  }

  /** NOTE:
   * 1. newController is not running (we would have returned in switchController already)
   * 2. newController can not be and emergency controller of the currently running controller
   * 3. newController could be being stopped by a different thread at the moment (wait for completion)
   */
  while(newController->isBeingStopped()){}

  //! initialize new controller
  newController->initializeController(timeStep_);

  // Set the newController as active controller as soon as the controller is initialized
  if ( newController->isControllerInitialized() ) {
    {
      //! This step has to be done when no update nor emergency stop is performed
      std::unique_lock<std::mutex> lockUpdate(updateControllerMutex_);
      std::unique_lock<std::mutex> lockEmergency(emergencyStopMutex_);
      // Protect also service calls accessing the active controller pair at the same time
      std::unique_lock<std::mutex> lockeActiveController(activeControllerMutex_);

      activeControllerPair_ = controllerPairs_.at(newController->getControllerName());

      // TODO check if state has changed during switch controller procedure then no setting of state::ok
      activeControllerState_ = State::OK;
    }

    // stop old controller
    if(oldController != nullptr) {
      oldController->stopController();
      oldController->setIsBeingStopped(false);
    }

    MELO_INFO("Switched to controller %s", activeControllerPair_.controller_->getControllerName().c_str());
  }
  else {
    // switch to freeze controller
    emergencyStop();
    ROS_INFO("Could not switch to controller %s", newController->getControllerName().c_str());
  }

  return true;
}

std::vector<std::string> ControllerManager::getAvailableControllerNames() {
  // fill vector of controller names
  std::vector<std::string> controllerNames;
  for( auto & controller : controllers_ )
  {
    controllerNames.push_back(controller.first);
  }

  return controllerNames;
}

std::string ControllerManager::getActiveControllerName() {
  std::string controllerName = "Failproof";
  std::lock_guard<std::mutex> activeControllerLock(activeControllerMutex_);

  switch(activeControllerState_) {
    case State::OK:
    {
      controllerName = activeControllerPair_.controller_->getControllerName();
      break;
    }
    case State::EMERGENCY:
    {
      controllerName = activeControllerPair_.emgcyController_->getControllerName();
      break;
    }
  }

  return controllerName;
}

bool ControllerManager::cleanup() {
  bool success = emergencyStop();

  std::unique_lock<std::mutex> lockController(controllerMutex_);
  std::unique_lock<std::mutex> lockEmergencyController(emergencyControllerMutex_);

  // cleanup all controllers
  // TODO wait for controllers to be finished with stopping or initializing -> no active workers
  {
    std::unique_lock<std::mutex> lockController(controllerMutex_);
    for (auto& controller : controllers_) {
      success = controller.second->cleanupController() && success;
    }
  }
  for (auto& emergency_controller : emergencyControllers_ ) {
    success = emergency_controller.second->cleanupController() && success;
  }

  return success;
}

bool ControllerManager::isRealRobot() const {
  return isRealRobot_;
}

void ControllerManager::setIsRealRobot(bool isRealRobot) {
  isRealRobot_ = isRealRobot;
}

//bool ControllerManager::checkTimingWorker(const any_worker::WorkerEvent& event){
//
//  static unsigned int wait_time_ms = timeStep_*minimalRealtimeFactor_*1000;
//
//  std::unique_lock<std::mutex> lk(updateFlagMutex_);
//  if( timerStart_.wait_for(lk, std::chrono::milliseconds(wait_time_ms)) == std::cv_status::timeout ||
//      timerStop_.wait_for(lk, std::chrono::milliseconds(wait_time_ms)) == std::cv_status::timeout) {
//    MELO_ERROR_STREAM("[CheckTiming]: Update controller took longer than: " << wait_time_ms << " ms. Emergency Stop!");
//    // Open new thread that perfoms emergency stop
//    // Print Warning
//    timerStop_.wait(lk);
//  }
//  return true;
//}

} /* namespace rocoma */
