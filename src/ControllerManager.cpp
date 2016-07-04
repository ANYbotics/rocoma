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
#include "rocoma/ControllerManager.hpp"

#include "message_logger/message_logger.hpp"

namespace rocoma {

ControllerManager::ControllerManager(std::unique_ptr<Controller> emergencyStopController,
                                     std::unique_ptr<Controller> fallbackController) :
                  timeStep_(0.0),
                  isRealRobot_(false),
                  controllers_(),
                  emergencyStopController_(std::move(emergencyStopController)),
                  fallbackController_(std::move(fallbackController)),
                  activeController_(nullptr)
{
  if(emergencyStopController_ == nullptr) {
    MELO_ERROR("Emergency controller is nullptr!");
    abort();
  }
}


ControllerManager::~ControllerManager()
{

}

bool ControllerManager::addController(std::unique_ptr<Controller> controller)  {

  // controller name (controller is moved within this function, therefore not safe to access everywhere)
  std::string controllerName = controller->getName();
  MELO_INFO("Adding controller %s ...", controllerName.c_str());

  // check if controller already exists
  auto ctrl = controllers_.find(controllerName);
  if(ctrl != controllers_.end()) {
    MELO_ERROR("Could not add controller %s. A controller with the same name already exists.", controllerName.c_str());
    return false;
  }

  // insert controller (move ownership to controller / controller is set to nullptr)
  using KV = std::pair<std::string, std::unique_ptr<Controller> >;
  controllers_.insert( KV(controllerName,std::unique_ptr<Controller>( std::move(controller) ) ) );


  // create controller
  if (!controllers_.at(controllerName)->createController(timeStep_)) {
    MELO_ERROR("Could not create controller %s!", controllerName.c_str());
    return false;
  }

  MELO_INFO("... finished adding controller %s.", controllerName.c_str());

  return true;
}

bool ControllerManager::updateController() {
  {
    std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);

    bool success = (*activeController_)->advanceController(timeStep_);

    if(fallbackController_ != nullptr) {
      success = fallbackController_->advanceController(timeStep_) && success;
    }

    return success;
  }
}

bool ControllerManager::emergencyStop() {
  {
    // TODO what to do with fallback controller
    std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);
    bool successPrestop = (*activeController_)->preStopController();
    bool successStop = (*activeController_)->stopController();
    return successPrestop && successStop;
  }
}

bool ControllerManager::switchToEmergencyController() {
  std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);

  if ( *activeController_ != emergencyStopController_) {
    activeController_ = &emergencyStopController_;
    return (*activeController_)->resetController(timeStep_);
  }

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
    std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);

    //! Check if controller is already active
    if (controllerName == (*activeController_)->getName()) {
      MELO_INFO("Controller %s is already running!", controllerName.c_str());
      return SwitchResponse::RUNNING;
    }
  }

  auto controller = controllers_.find(controllerName);
  if(controller != controllers_.end()) {
    // store pointers to old and new controllers
    std::unique_ptr<Controller>* newController = &controller->second;
    std::unique_ptr<Controller>* oldController = activeController_;

    // shutdown communication for active controller
    {
      std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);
      (*oldController)->preStopController();
    }

    // initialize new controller
    (*newController)->initializeController(timeStep_);

    if ( (*newController)->isInitialized() ) {
      {
        // set active controller
        std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);
        activeController_ = newController;
      }

      // swap out old controller
      //(*oldController)->swapOut();

      MELO_INFO("Switched to controller %s", (*activeController_)->getName().c_str());
      response = SwitchResponse::SWITCHED;

    }
    else {
      // switch to freeze controller TODO: react if can't switch to emergency state?
      switchToEmergencyController();
      ROS_INFO("Could not switch to controller %s", (*newController)->getName().c_str());
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
  std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);

  // fill vector of controller names
  std::vector<std::string> controllerNames;
  for( auto & controller : controllers_ )
  {
    controllerNames.push_back(controller.first);
  }

  return controllerNames;
}

std::string ControllerManager::getActiveControllerName() {
  std::lock_guard<std::recursive_mutex> lock(activeControllerMutex_);
  return (*activeController_)->getName();
}

bool ControllerManager::isRealRobot() const {
  return isRealRobot_;
}

void ControllerManager::setIsRealRobot(bool isRealRobot) {
  isRealRobot_ = isRealRobot;
}

} /* namespace rocoma */
