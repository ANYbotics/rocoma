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

// rocoma
#include "rocoma/ControllerManager.hpp"

// Message logger
#include "message_logger/message_logger.hpp"

// STL
#include <limits>
#include <algorithm>

namespace rocoma {

ControllerManager::ControllerManager(const double timestep,
                                     const bool isRealRobot,
                                     const LoggerOptions& loggerOptions):
  ControllerManager(ControllerManagerOptions{timestep, isRealRobot, loggerOptions})

{
  //  any_worker::WorkerOptions checkTimingWorkerOptions;
  //  checkTimingWorkerOptions.name_ = "check_timing";
  //  checkTimingWorkerOptions.timeStep_ = 0;
  //  checkTimingWorkerOptions.callback_ = std::bind(&ControllerManager::checkTimingWorker, this, std::placeholders::_1);
  //  checkTimingWorkerOptions.defaultPriority_ = 0;
  //  checkTimingWorkerOptions.destructWhenDone_ = false;
  //  workerManager_.addWorker(checkTimingWorkerOptions, true);
}

/**
 * @brief Constructor
 * @param options Configuration Options of the manager
 */
ControllerManager::ControllerManager(const ControllerManagerOptions & options):
//    updating_(false),
//    timerStart_(),
//    timerStop_(),
//    minimalRealtimeFactor_(2.0),
  isInitialized_(true),
  options_(options),
  emergencyStopMustBeCleared_(false),
  hasClearedEmergencyStop_(true),
  activeControllerState_(State::FAILURE),
  workerManager_(),
  controllers_(),
  emergencyControllers_(),
  sharedModules_(),
  controllerPairs_(),
  activeControllerPair_(nullptr, nullptr),
  failproofController_(nullptr),
  controllerMutex_(),
  emergencyControllerMutex_(),
  failproofControllerMutex_(),
  emergencyStopMutex_(),
  updateControllerMutex_(),
  switchControllerMutex_(),
  workerManagerMutex_(),
  activeControllerMutex_()
{

}

ControllerManager::ControllerManager():
        ControllerManager(0.01, false, LoggerOptions())
{
  // Hack
  isInitialized_ = false;
}

void ControllerManager::init(const ControllerManagerOptions & options)
{
  if (isInitialized_) {
    MELO_WARN("[Rocoma] Controller Manager was already initialized. Do nothing.");
    return;
  }
  options_ = options;
  isInitialized_ = true;
}

bool ControllerManager::addControllerPair(ControllerPtr&& controller,
                                          EmgcyControllerPtr&& emergencyController) {
  if(!isInitialized_) {
    MELO_ERROR("[Rocoma] Controller Manager is not initialized. Can not add controller pair.");
    return false;
  }

  // Add controller
  if(!createController(controller)) {
    return false;
  }

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();
  const std::string emgcyControllerName = (emergencyController == nullptr)?"FailproofController": emergencyController->getControllerName();

  // insert controller (move ownership to controller / controller is set to nullptr)
  controllers_.insert( std::pair<std::string, ControllerPtr >(controllerName, std::move(controller) ) );
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Successfully added controller!");

  //--- Add emergency controller
  MELO_DEBUG_STREAM("[Rocoma][" << emgcyControllerName << "] Add emergency controller!");

  if(emergencyController == nullptr)
  {
    controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName,
                                                                         ControllerSetPtr(controllers_.at(controllerName).get(), nullptr ) ) );
    MELO_INFO_STREAM("[Rocoma][" << controllerName << " / " << emgcyControllerName << "] Successfully added controller pair.");
    return true;
  }

  // set properties
  emergencyController->setIsRealRobot(options_.isRealRobot);

  // create emergency controller
  if (!emergencyController->createController(options_.timeStep)) {
    MELO_WARN_STREAM("[Rocoma][" << emgcyControllerName << "] Could not be created! Use failproof controller on emergency stop!");
    controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName,
                                                                         ControllerSetPtr(controllers_.at(controllerName).get(), nullptr) ) );
    return false;
  }

  // check if emergency controller already exists
  if(emergencyControllers_.find(emgcyControllerName) != emergencyControllers_.end()) {
    MELO_INFO_STREAM("[Rocoma][" << emgcyControllerName << "] An emergency controller with the name already exists. Using same instance.");
  }
  else {
    // insert emergency controller (move ownership to controller / controller is set to nullptr)
    emergencyControllers_.insert( std::pair<std::string, EmgcyControllerPtr>(emgcyControllerName, std::move(emergencyController) ) );
    MELO_DEBUG_STREAM("[Rocoma][" << emgcyControllerName << "] Successfully added emergency controller!");
  }

  // Add controller pair
  controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName,
                                                                       ControllerSetPtr(controllers_.at(controllerName).get(), emergencyControllers_.at(emgcyControllerName).get() ) ) );
  MELO_INFO_STREAM("[Rocoma][" << controllerName << " / " << emgcyControllerName << "] Successfully added controller pair.");

  return true;
}

bool ControllerManager::addControllerPairWithExistingEmergencyController(ControllerPtr&& controller,
                                                                         const std::string & emgcyControllerName)
{
  if(!isInitialized_) {
    MELO_ERROR("[Rocoma] Controller Manager is not initialized. Can not add controller with existing emergency controller.");
    return false;
  }

  // Add controller
  if(!createController(controller)) { return false; };

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();

  // insert controller (move ownership to controller / controller is set to nullptr)
  controllers_.insert( std::pair<std::string, ControllerPtr >(controllerName, std::move(controller) ) );
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Successfully added controller!");

  // check if emergency controller already exists
  if(emergencyControllers_.find(emgcyControllerName) != emergencyControllers_.end()) {
    MELO_INFO_STREAM("[Rocoma][" << emgcyControllerName << "] An emergency controller with the name already exists. Using same instance.");
    controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName,
                                                                         ControllerSetPtr(controllers_.at(controllerName).get(), emergencyControllers_.at(emgcyControllerName).get()) ) );
    MELO_INFO_STREAM("[Rocoma][" << controllerName << " / " << emgcyControllerName << "] Successfully added controller pair.");
  }
  else {
    MELO_WARN_STREAM("[Rocoma][" << emgcyControllerName << "] Does not exist in list! Use failproof controller on emergency stop!");
    controllerPairs_.insert( std::pair< std::string, ControllerSetPtr >( controllerName,
                                                                         ControllerSetPtr(controllers_.at(controllerName).get(), nullptr) ) );
    MELO_INFO_STREAM("[Rocoma][" << controllerName << " / ] Successfully added controller pair.");
  }

  return true;
}


bool ControllerManager::setFailproofController(FailproofControllerPtr&& controller)
{
  if(!isInitialized_) {
    MELO_ERROR("[Rocoma] Controller Manager is not initialized. Can not set failproof controller.");
    return false;
  }

  // If nullptr abort
  if(controller == nullptr) {
    MELO_ERROR_STREAM("[Rocoma] Could not add failproof controller. Failproof controller is nullptr. Abort!");
    exit(-1);
  }

  // controller name (controller is moved within this function, therefore not safe to access everywhere)
  std::string controllerName = controller->getControllerName();
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Adding failproof controller!");

  // create controller
  if (!controller->createController(options_.timeStep)) {
    MELO_ERROR_STREAM("[Rocoma][" << controllerName << "] Could not create failproof controller. Abort!");
    exit(-1);
  }

  // move controller
  failproofController_ = std::move(controller);
  MELO_INFO_STREAM("[Rocoma][" << controllerName << "] Successfully added failproof controller!");

  return true;
}

bool ControllerManager::updateController() {

  if(failproofController_.get() == nullptr) {
    MELO_ERROR("[Rocoma] Can not advance controller manager. Failproof controller is null. Abort!");
    exit(-1);
  }

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
    if(!activeControllerPair_.controller_->advanceController(options_.timeStep))
    {
      lockController.unlock();
      return emergencyStop();
    }
  }

  // Controller is in emergency stop
  if(activeControllerState_ == State::EMERGENCY)
  {
    std::unique_lock<std::mutex> lockEmergencyController(emergencyControllerMutex_);
    if(!activeControllerPair_.emgcyController_->advanceController(options_.timeStep))
    {
      lockEmergencyController.unlock();
      return emergencyStop();
    }
  }

  // Failproof controller is active
  if(activeControllerState_ == State::FAILURE)
  {
    // returns void -> can never fail!
    std::unique_lock<std::mutex> lockFailproofCOntroller(failproofControllerMutex_);
    failproofController_->advanceController(options_.timeStep);
  }

  //  // stop update
  //  updating_ = false;
  //  timerStop_.notify_one();

  return true;
}

bool ControllerManager::emergencyStop() {
  // Forbid controller switches
  if(emergencyStopMustBeCleared_){
    hasClearedEmergencyStop_.store(false);
  }

  MELO_ERROR("[Rocoma] Emergency Stop!");
  // Cannot call emergency stop twice simultaneously
  std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);

  // Clean workers
  {
    std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
    workerManager_.cleanDestructibleWorkers();
  }

  // Check if controller is in failproof state already
  if(activeControllerState_ == State::FAILURE)
  {
    MELO_DEBUG("[Rocoma] Failproof controller is already running on emergency stop!");
    return false;
  }

  // Stop worker options
  any_worker::WorkerOptions stopWorkerOptions;
  stopWorkerOptions.timeStep_ = std::numeric_limits<double>::infinity();
  stopWorkerOptions.defaultPriority_ = 0;
  stopWorkerOptions.destructWhenDone_ = true;

  // If state ok and emergency controller registered -> switch to emergency controller
  if(activeControllerState_ == State::OK) {
    // stop controller in a different thread
    stopWorkerOptions.name_ = "stop_controller_" + activeControllerPair_.controllerName_;
    stopWorkerOptions.callback_ = std::bind(&ControllerManager::emergencyStopControllerWorker, this, std::placeholders::_1,
                                            activeControllerPair_.controller_, EmergencyStopType::EMERGENCY);
    {
      std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
      workerManager_.addWorker(stopWorkerOptions, true);
    }

    if(options_.loggerOptions.enable) {
      // Save logger data
      signal_logger::logger->stopLogger();
      signal_logger::logger->saveLoggerData( options_.loggerOptions.fileTypes );
    }

    if(activeControllerPair_.emgcyController_ != nullptr &&
        !activeControllerPair_.emgcyController_->isBeingStopped()) {

      bool success = true;

      {
        std::unique_lock<std::mutex> lockEmergencyController(emergencyControllerMutex_);
        // Init emergency controller fast
        success = activeControllerPair_.emgcyController_->initializeControllerFast(options_.timeStep);
        // only advance if correctly initialized
        success = success && activeControllerPair_.emgcyController_->advanceController(options_.timeStep);
      }

      if(success)
      {
        // Start logger
        if(options_.loggerOptions.enable){
          signal_logger::logger->startLogger(options_.loggerOptions.updateOnStart);
        }

        // Switch to emergency state
        activeControllerState_ = State::EMERGENCY;
        this->notifyControllerChanged(activeControllerPair_.emgcyControllerName_);
        this->notifyControllerManagerStateChanged(activeControllerState_.load());

        // Return here -> do not move on to failproof controller
        return true;
      }
    }

  }
  else {
    // stop emergency controller in a different thread
    stopWorkerOptions.name_ = "stop_controller_" + activeControllerPair_.emgcyControllerName_;
    stopWorkerOptions.callback_ =
        std::bind(&ControllerManager::emergencyStopControllerWorker, this, std::placeholders::_1,
                  activeControllerPair_.emgcyController_, EmergencyStopType::FAILPROOF);
    {
      std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
      workerManager_.addWorker(stopWorkerOptions, true);
    }

    if (options_.loggerOptions.enable) {
      // Stop the logger
      signal_logger::logger->stopLogger();
      signal_logger::logger->saveLoggerData(options_.loggerOptions.fileTypes);
    }
  }
  // Advance failproof controller
  {
    MELO_INFO("[Rocoma] Switched to failproof controller!");
    std::unique_lock<std::mutex> lockFailproofController(failproofControllerMutex_);
    failproofController_->advanceController(options_.timeStep);
  }

  // Switch to failure state
  activeControllerState_ = State::FAILURE;
  this->notifyControllerChanged(failproofController_->getControllerName());
  this->notifyControllerManagerStateChanged(activeControllerState_.load());

  return true;
}

void ControllerManager::clearEmergencyStop() {
  MELO_INFO("[Rocoma] Cleared Emergency Stop.");
  hasClearedEmergencyStop_.store(true);
}

ControllerManager::SwitchResponse ControllerManager::switchController(const std::string & controllerName) {
  // init promise and future
  std::promise<SwitchResponse> switch_promise;
  std::future<SwitchResponse> switch_future = switch_promise.get_future();

  // switch controller
  this->switchController(controllerName, std::ref(switch_promise));

  // wait for future result
  switch_future.wait();

  return switch_future.get();
}

void ControllerManager::switchController(const std::string & controllerName,
                                         std::promise<SwitchResponse> & response_promise) {

  // Allow only sequential calls to switch controller
  std::unique_lock<std::mutex> lockSwitchController(switchControllerMutex_);

  // Make sure were not in emergency stop procedure
  {
    std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);
  }

  // Check if controller is already active
  {
    std::unique_lock<std::mutex> lockActiveController(activeControllerMutex_);
    if (activeControllerState_ == State::OK && controllerName == activeControllerPair_.controllerName_) {
      MELO_INFO("[Rocoma] Controller %s is already running!", controllerName.c_str());
      response_promise.set_value(SwitchResponse::RUNNING);
      return;
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
                                                            std::placeholders::_1, activeControllerPair_.controller_, controllerPair->second.controller_, std::ref(response_promise));
        break;
      }
      case State::EMERGENCY:
      {
        switchControllerWorkerOptions.name_ = "switch_from_" + activeControllerPair_.emgcyControllerName_ + "_to_" + controllerPair->second.controllerName_;
        switchControllerWorkerOptions.callback_ = std::bind(&ControllerManager::switchControllerWorker, this,
                                                            std::placeholders::_1, activeControllerPair_.emgcyController_, controllerPair->second.controller_, std::ref(response_promise));
        break;
      }
      case State::FAILURE:
      {
        switchControllerWorkerOptions.name_ = "switch_from_failproof_to_" + controllerPair->second.controller_->getControllerName();
        switchControllerWorkerOptions.callback_ = std::bind(&ControllerManager::switchControllerWorker, this,
                                                            std::placeholders::_1, nullptr, controllerPair->second.controller_, std::ref(response_promise));
        break;
      }
    }

    {
      // Add worker to switch to new controller
      std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
      workerManager_.cleanDestructibleWorkers();
      if( !workerManager_.addWorker(switchControllerWorkerOptions, true) ) {
        MELO_ERROR_STREAM("[Rocoma] Can not create worker! Already running a switch thread with the same name?");
        response_promise.set_value(SwitchResponse::ERROR);
      }
    }

    return;
  }
  else {
    // controller is not part of controller map
    MELO_INFO("[Rocoma] Controller %s not found!", controllerName.c_str());
    response_promise.set_value(SwitchResponse::NOTFOUND);
    return;
  }

  response_promise.set_value(SwitchResponse::ERROR);
  return;
}

std::vector<std::string> ControllerManager::getAvailableControllerNames() {
  // fill vector of controller names
  std::vector<std::string> controllerNames;
  for( auto & controller : controllers_ )
  {
    controllerNames.push_back(controller.first);
  }

  // Sort them alphabetically
  std::sort(controllerNames.begin(), controllerNames.end(), std::less<std::string>());

  return controllerNames;
}

std::string ControllerManager::getActiveControllerName() {
  std::string controllerName = "-";
  std::lock_guard<std::mutex> activeControllerLock(activeControllerMutex_);

  switch(activeControllerState_) {
    case State::OK:
    {
      controllerName = activeControllerPair_.controllerName_;
      break;
    }
    case State::EMERGENCY:
    {
      controllerName = activeControllerPair_.emgcyControllerName_;
      break;
    }
    case State::FAILURE:
    {
      controllerName = "Failproof";
      break;
    }
  }

  return controllerName;
}

ControllerManager::State ControllerManager::getControllerManagerState() {
  return activeControllerState_;
}

bool ControllerManager::cleanup() {
  bool success = true;


  while(activeControllerState_ != State::FAILURE)
  {
    success = emergencyStop() && success;
  }

  {
    std::unique_lock<std::mutex> lockController(controllerMutex_);
    std::unique_lock<std::mutex> lockEmergencyController(emergencyControllerMutex_);

    // stop all workers
    MELO_DEBUG("[Rocoma] Stopping all workers.");
    {
      std::unique_lock<std::mutex> lockWorkerManager(workerManagerMutex_);
      workerManager_.stopWorkers(true);
    }

    // cleanup all controllers
    // TODO wait for controllers to be finished initializing
    MELO_DEBUG("[Rocoma] Cleaning all controllers up.");
    for (auto& controller : controllers_) {
      while(controller.second->isBeingStopped()) { MELO_INFO_THROTTLE_STREAM(1.0, "Stopping controller " << controller.first ); }
      success = controller.second->cleanupController() && success;
      // clean up unique ptrs here.
      // They are managed by ControllerManager and are pointing to instances classes found in dynamically loaded libraries.
      // The libraries are loaded and managed by the child class ControllerManagerRos. The destructor of ControllerManagerRos is called before
      // the destructor of ControllerManager, cleaning up the loaded libraries. This leaves these unique_ptrs pointing to an instance of an unknown class.
      controller.second.reset(nullptr);
    }

    MELO_DEBUG("[Rocoma] Cleaning all emergency controllers up.");
    for (auto& emergency_controller : emergencyControllers_ ) {
      while(emergency_controller.second->isBeingStopped()) { }
      success = emergency_controller.second->cleanupController() && success;
      emergency_controller.second.reset(nullptr); // clean up unique ptrs here, see above
    }
  }
  MELO_DEBUG("[Rocoma] Reset fail proof controller.");
  failproofController_->cleanupController();
  failproofController_.reset(nullptr); // clean up unique ptrs here, see above

  return success;
}

bool ControllerManager::createController(const ControllerPtr & controller) {

  // Check for invalid controller
  if(controller == nullptr) {
    MELO_ERROR_STREAM("[Rocoma] Could not add controller pair. Controller is nullptr.");
    return false;
  }

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();

  // check if controller already exists
  if(controllers_.find(controllerName) != controllers_.end()) {
    MELO_WARN_STREAM("[Rocoma][" << controllerName << "] Could not add controller. A controller with the same name already exists.");
    return false;
  }

  //--- Add controller
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Adding controller!");

  // Set controller properties
  controller->setIsRealRobot(options_.isRealRobot);

  // create controller
  if (!controller->createController(options_.timeStep)) {
    MELO_ERROR_STREAM("[Rocoma][" << controllerName << "] Could not create controller!");
    return false;
  }

  return true;
}

bool ControllerManager::emergencyStopControllerWorker(const any_worker::WorkerEvent& e,
                                                      roco::ControllerAdapterInterface * controller,
                                                      EmergencyStopType emgcyStopType)
{
  bool success = true;

  // notify emergency stop
  notifyEmergencyStop(emgcyStopType);

  {
    // Stop controller and block -> switch controller can not happen while controller is stopped
    controller->setIsBeingStopped(true);
    success = controller->preStopController();
    success = controller->stopController() && success;
    controller->setIsBeingStopped(false);
  }

  return success;
}

bool ControllerManager::switchControllerWorker(const any_worker::WorkerEvent& e,
                                               roco::ControllerAdapterInterface * oldController,
                                               roco::ControllerAdapterInterface * newController,
                                               std::promise<SwitchResponse> & response_promise) {
  /** NOTE:
   * 1. The active controller is not blocked -> by definition there can be no data races between advance and preStop
   * 2. Set this oldController to beeing stopped to prevent a switch to it
   */
  // shutdown communication for active controller
  if(oldController != nullptr) {
    oldController->setIsBeingStopped(true);
    oldController->preStopController();
  }

  // Stop logger if running
  if(options_.loggerOptions.enable && signal_logger::logger->isRunning()) {
    // Save logger data
    signal_logger::logger->stopLogger();
    signal_logger::logger->saveLoggerData( options_.loggerOptions.fileTypes );
  }

  /** NOTE:
   * 1. newController is not running (we would have returned in switchController already)
   * 2. newController can not be and emergency controller of the currently running controller
   * 3. newController could be being stopped by a different thread at the moment (wait for completion)
   */
  if(newController->isBeingStopped()) {
    MELO_WARN_STREAM("[Rocoma][" << newController->getControllerName() << "] Controller is currently being stopped. Wait for completion before switching.")
  }
  while(newController->isBeingStopped()){}

  //! initialize new controller
  roco::ControllerSwapStateInterfacePtr state(nullptr);
  if(oldController != nullptr) { oldController->getControllerSwapState(state); }

  if(hasClearedEmergencyStop()) {
    if(!newController->swapController(options_.timeStep, state)) {
      MELO_ERROR_STREAM("[Rocoma][" << newController->getControllerName() << "] Could not swap. Not switching.");
      response_promise.set_value(SwitchResponse::ERROR);
      return false;
    }
  } else {
    MELO_ERROR_STREAM("[Rocoma][" << newController->getControllerName() << "] Could not swap. Emergency stop was not cleared. Not switching.");
    response_promise.set_value(SwitchResponse::ERROR);
    return false;
  }

  // Start Logging
  if(options_.loggerOptions.enable){
    signal_logger::logger->startLogger(options_.loggerOptions.updateOnStart);
  }

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

    MELO_INFO("[Rocoma] Switched to controller %s", activeControllerPair_.controllerName_.c_str());
    this->notifyControllerChanged(activeControllerPair_.controllerName_);
    this->notifyControllerManagerStateChanged(activeControllerState_.load());
    response_promise.set_value(SwitchResponse::SWITCHING);
    return true;
  }
  else {
    // switch to freeze controller
    emergencyStop();
    MELO_ERROR_STREAM("[Rocoma][" << newController->getControllerName() << "] Controller initialization was unsuccessful. Not switching.");
    response_promise.set_value(SwitchResponse::ERROR);
    return false;
  }

  return true;
}

bool ControllerManager::addSharedModule(roco::SharedModulePtr&& sharedModule) {
  std::string name = sharedModule->getName();

  if( hasSharedModule(name) ) {
    MELO_WARN_STREAM("[Rocoma][" << name << "] A shared module with this name was already added! Do nothing.");
    return true;
  }

  sharedModules_.emplace(name, std::move(sharedModule));

  return true;
}



bool ControllerManager::hasSharedModule(const std::string & moduleName) {
  return sharedModules_.find(moduleName) != sharedModules_.end();
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
