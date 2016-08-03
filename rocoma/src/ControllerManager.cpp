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

namespace rocoma {

ControllerManager::ControllerManager() :
                                                  timeStep_(0.1),
                                                  isRealRobot_(false),
                                                  controllers_(),
                                                  emergencyControllers_(),
                                                  failproofController_(nullptr),
                                                  activeControllerPair_(nullptr, nullptr),
                                                  activeControllerState_(ManagedControllerState::OK)
{

}


ControllerManager::~ControllerManager()
{

}

bool ControllerManager::addControllerPair(std::unique_ptr<roco::ControllerAdapterInterface> controller,
                                          std::unique_ptr<roco::EmergencyControllerAdapterInterface> emergencyController) {

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();
  const std::string emgcyControllerName = emergencyController->getControllerName();

  MELO_INFO_STREAM("Adding controller pair ctrl: " << controllerName << " / emgcy ctrl: " << emgcyControllerName << " ... ");

  // Add controller
  MELO_INFO_STREAM(" Adding controller " << controllerName << " ... ");

  // create controller
  if (!controller->createController(timeStep_)) {
    MELO_ERROR_STREAM("... Could not create controller " << controllerName << "!");
    return false;
  }
  if (!controller->initializeController(timeStep_)) {
    MELO_ERROR_STREAM("... Could not create controller " << controllerName << "!");
    return false;
  }

  // check if controller already exists
  if(controllers_.find(controllerName) != controllers_.end()) {
    MELO_WARN_STREAM("... Could not add controller " << controllerName << ". A controller with the same name already exists.");
    return false;
  }
  else {
    // insert controller (move ownership to controller / controller is set to nullptr)
    using KV = std::pair<std::string, ControllerPtr >;
    controllers_.insert( KV(controllerName, ControllerPtr( std::move(controller) ) ) );
    MELO_INFO_STREAM("... successfully added controller " << controllerName << "!");
  }

  // Add emergency controller
  MELO_INFO_STREAM(" Adding emergency controller " << emgcyControllerName << " ... ");

  // create emergency controller
  if (!emergencyController->createController(timeStep_)) {
    MELO_ERROR_STREAM("... Could not create emergency controller " << emgcyControllerName << "! Use failproof controller on emergency stop!");
    using KV = std::pair< std::string, ControllerPtrPtrPair >;
    controllerPairs_.insert( KV( controllerName, ControllerPtrPtrPair(&controllers_.at(controllerName), nullptr) ) );
    return false;
  }
  if (!emergencyController->initializeController(timeStep_)) {
    MELO_ERROR_STREAM("... Could not create controller " << controllerName << "!");
    return false;
  }

  // check if emergency controller already exists
  if(emergencyControllers_.find(emgcyControllerName) != emergencyControllers_.end()) {
    MELO_WARN_STREAM("... Could not add emergency controller " << emgcyControllerName << ". An emergency controller with the same name already exists.");
  }
  else {
    // insert emergency controller (move ownership to controller / controller is set to nullptr)
    using KV = std::pair<std::string, EmgcyControllerPtr>;
    emergencyControllers_.insert( KV(emgcyControllerName, EmgcyControllerPtr( std::move(emergencyController) ) ) );
    MELO_INFO_STREAM("... successfully added emergency controller " << emgcyControllerName << "!");
  }

  // Add controller pair and set active
  controllerPairs_.insert( std::pair< std::string, ControllerPtrPtrPair >( controllerName,
                                                                           ControllerPtrPtrPair( &controllers_.at(controllerName), &emergencyControllers_.at(emgcyControllerName) ) ) );
  activeControllerPair_ = controllerPairs_.at(controllerName);
  MELO_INFO_STREAM("... sucessfully added controller pair ctrl: " << controllerName << " / emgcy ctrl: " << emgcyControllerName << " ... ");

  return true;
}

bool ControllerManager::setFailproofController(std::unique_ptr<roco::FailproofControllerAdapterInterface> controller)  {

  // controller name (controller is moved within this function, therefore not safe to access everywhere)
  std::string controllerName = controller->getControllerName();
  MELO_INFO("Adding failproof controller %s ...", controllerName.c_str());

  failproofController_ = std::unique_ptr<roco::FailproofControllerAdapterInterface>( std::move(controller) );

  // create controller
  if (!failproofController_->createController(timeStep_)) {
    MELO_ERROR("Could not create failproof controller %s!", controllerName.c_str());
    return false;
  }

  MELO_INFO("... finished adding failproof controller %s.", controllerName.c_str());

  return true;
}

bool ControllerManager::updateController() {
  std::unique_lock<std::mutex> ctrlLock(activeControllerMutex_);

  joinStopControllerThreads();

  // Controller is running
  if(activeControllerState_ == ManagedControllerState::OK)
  {
    // Advance "normal" controller -> if advance return false treat as emergency stop
    if(!(*activeControllerPair_.controller_)->advanceController(timeStep_))
    {
      if(activeControllerPair_.emgcyController_ != nullptr)
      {
        emergencyStop();
      }
      else {
        emergencyStop();
      }
    }
  }

  // Controller is in emergency stop
  if(activeControllerState_ == ManagedControllerState::EMERGENCY)
  {
    if(!(*activeControllerPair_.emgcyController_)->advanceController(timeStep_))
    {
      // Use failproof controller if emergency stop fails
      activeControllerState_ = ManagedControllerState::FAILURE;
    }
  }

  // Failproof controller is active
  if(activeControllerState_ == ManagedControllerState::FAILURE)
  {
    // returns void -> can never fail!
    failproofController_->advanceController(timeStep_);
  }

  return true;
}

bool ControllerManager::stopController(roco::ControllerAdapterInterface * controller) {
  std::lock_guard<std::mutex> lock(stopThreadsMutex_);
  MELO_WARN_STREAM("Adding thread.");
  stoppingThreads_.push_back( StopControllerThread( controller, std::thread(&roco::ControllerAdapterInterface::stopController, controller) ) );
  return true;
}

void ControllerManager::joinStopControllerThreads() {
  std::lock_guard<std::mutex> lock(stopThreadsMutex_);

  if(stoppingThreads_.size()) {
  if(stoppingThreads_.begin()->done())
  {
    if(stoppingThreads_.begin()->thread_.joinable()) {
      stoppingThreads_.begin()->thread_.join();
      MELO_WARN_STREAM("Joined the thread stopping controller " << stoppingThreads_.begin()->controller_->getControllerName() << "!");
    }
    stoppingThreads_.erase(stoppingThreads_.begin());
  }
  }
  return;
}

bool ControllerManager::emergencyStop() {
  std::lock_guard<std::mutex> lock(activeControllerMutex_);

  if(activeControllerState_ == ManagedControllerState::OK && activeControllerPair_.emgcyController_ != nullptr) {
    // Go to emergency state and fast initialize controller
    activeControllerState_ = ManagedControllerState::EMERGENCY;

    stopController(activeControllerPair_.controller_->get());

    reactOnEmergencyStop( EmergencyStopObserver::EmergencyStopType::SMART_EMERGENCY_STOP );
  }
  else {
    // Go to emergency state and fast initialize controller
    activeControllerState_ = ManagedControllerState::FAILURE;
    stopController(activeControllerPair_.emgcyController_->get());
    reactOnEmergencyStop( EmergencyStopObserver::EmergencyStopType::FAILPROOF_EMERGENCY_STOP );
  }

  return true;
  //  {
  //    std::lock_guard<std::mutex> lock(activeControllerMutex_);
  //    bool successPrestop = (*activeControllerPair_.first)->preStopController();
  //    bool successStop = (*activeControllerPair_.first)->stopController();
  //    return successPrestop && successStop;
  //  }
}

bool ControllerManager::switchToEmergencyController() {
  std::lock_guard<std::mutex> lock(activeControllerMutex_);

  //  if ( *activeController_ != emergencyStopController_) {
  //    activeController_ = &emergencyStopController_;
  //    return (*activeController_)->resetController(timeStep_);
  //  }

  return true;
}

bool ControllerManager::cleanup() {
  bool success = emergencyStop();

  // cleanup all controllers
  for (auto& controller : controllers_) {
    success = controller.second->cleanupController() && success;
  }

  return success;
}

ControllerManager::SwitchResponse ControllerManager::switchController(const std::string & controllerName) {

  SwitchResponse response = SwitchResponse::ERROR;
  {
    std::lock_guard<std::mutex> lock(activeControllerMutex_);

    //! Check if controller is already active
    if (controllerName == (*activeControllerPair_.controller_)->getControllerName()) {
      MELO_INFO("Controller %s is already running!", controllerName.c_str());
      return SwitchResponse::RUNNING;
    }
  }

  auto controllerPair = controllerPairs_.find(controllerName);
  if(controllerPair != controllerPairs_.end()) {
    // store pointers to old and new controllers
    std::unique_ptr<roco::ControllerAdapterInterface>* newController = controllerPair->second.controller_;
    std::unique_ptr<roco::ControllerAdapterInterface>* oldController = activeControllerPair_.controller_;

    // shutdown communication for active controller
    {
      std::lock_guard<std::mutex> lock(activeControllerMutex_);
      (*oldController)->preStopController();
    }

    // initialize new controller
    (*newController)->initializeController(timeStep_);

    if ( (*newController)->isControllerInitialized() ) {
      {
        // set active controller
        std::lock_guard<std::mutex> lock(activeControllerMutex_);
        activeControllerPair_.controller_ = newController;
      }

      // swap out old controller
      (*oldController)->stopController();

      MELO_INFO("Switched to controller %s", (*activeControllerPair_.controller_)->getControllerName().c_str());
      response = SwitchResponse::SWITCHED;

    }
    else {
      // switch to freeze controller TODO: react if can't switch to emergency state?
      switchToEmergencyController();
      ROS_INFO("Could not switch to controller %s", (*newController)->getControllerName().c_str());
      response = SwitchResponse::ERROR;
    }
  }
  else {
    // controller is not part of controller map
    MELO_INFO("Controller %s not found!", controllerName.c_str());
    response = SwitchResponse::NOTFOUND;
  }

  return response;

}

std::vector<std::string> ControllerManager::getAvailableControllerNames() {
  // TODO: Is this lock needed?
  std::lock_guard<std::mutex> lock(activeControllerMutex_);

  // fill vector of controller names
  std::vector<std::string> controllerNames;
  for( auto & controller : controllers_ )
  {
    controllerNames.push_back(controller.first);
  }

  return controllerNames;
}

std::string ControllerManager::getActiveControllerName() {
  std::lock_guard<std::mutex> lock(activeControllerMutex_);
  return (*activeControllerPair_.controller_)->getControllerName();
}

bool ControllerManager::isRealRobot() const {
  return isRealRobot_;
}

void ControllerManager::setIsRealRobot(bool isRealRobot) {
  isRealRobot_ = isRealRobot;
}

} /* namespace rocoma */
