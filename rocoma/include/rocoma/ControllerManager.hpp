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
 * @author  Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date    Oct, 2014
 */

#pragma once

// roco
#include <roco/controllers/adapters/ControllerAdapterInterface.hpp>
#include <roco/controllers/adapters/EmergencyControllerAdapterInterface.hpp>
#include <roco/controllers/adapters/FailproofControllerAdapterInterface.hpp>

// rocoma
#include <rocoma/common/EmergencyStopObserver.hpp>

// Boost
#include <boost/ptr_container/ptr_unordered_map.hpp>

// STL
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <thread>
#include <mutex>

namespace rocoma {

struct StopControllerThread {

  StopControllerThread(roco::ControllerAdapterInterface * ctrl,
                       std::thread thread):
                         controller_(ctrl),
                         thread_(std::move(thread))
  {

  }

  bool done() {
    return !controller_->isStopping();
  }

  roco::ControllerAdapterInterface* controller_;
  std::thread thread_;

};


struct ControllerPtrPtrPair {

  ControllerPtrPtrPair(std::unique_ptr<roco::ControllerAdapterInterface> * ctrl,
                       std::unique_ptr<roco::EmergencyControllerAdapterInterface> * emgcyCtrl):
                         controller_(ctrl),
                         emgcyController_(emgcyCtrl)
  {

  }

  std::unique_ptr<roco::ControllerAdapterInterface> * controller_;
  std::unique_ptr<roco::EmergencyControllerAdapterInterface> * emgcyController_;

};

class ControllerManager
{
 public:
  //! Enumeration for switch controller feedback
  enum class SwitchResponse : int {
    NOTFOUND = -2,
        ERROR    = -1,
        RUNNING  =  0,
        SWITCHED =  1
  };

  enum class ManagedControllerState : int {
    FAILURE = -1,
        EMERGENCY = 0,
        OK = 1
  };

  using ControllerPtr = std::unique_ptr<roco::ControllerAdapterInterface>;
  using EmgcyControllerPtr = std::unique_ptr<roco::EmergencyControllerAdapterInterface>;
  using FailproofControllerPtr = std::unique_ptr<roco::FailproofControllerAdapterInterface>;

 public:
  //! Delete default constructor
  ControllerManager();

  //! Destructor
  virtual ~ControllerManager();

  /**
   * @brief Add a controller to the controllers map
   * @param controller    Pointer to the controller (unique ptr -> ownership transfer)
   * @return true, if controller was created and added successfully
   */
  bool addControllerPair(std::unique_ptr<roco::ControllerAdapterInterface> controller,
                         std::unique_ptr<roco::EmergencyControllerAdapterInterface> emergencyController);

  bool setFailproofController(std::unique_ptr<roco::FailproofControllerAdapterInterface> controller);

  /**
   * @brief Advance the currently active controller
   * @return true, if controller was advanced successfully
   */
  bool updateController();

  bool stopController(roco::ControllerAdapterInterface * stopController);
  void joinStopControllerThreads();

  /**
   * @brief Prestop and stop controller
   * @return true, if both stopping procedures where successful
   */
  bool emergencyStop();

  /**
   * @brief Switch to emergency stop controller
   * @return true, if emergency stop controller is already running or reset properly
   */
  bool switchToEmergencyController();

  /**
   * @brief Cleanup all controllers
   * @return true, if successful emergency stop and all controllers are cleaned up
   */
  bool cleanup();

  /**
   * @brief Tries to switch to a desired controller
   * @param controllerName    Name of the desired controller
   * @return result of the switching operation
   */
  SwitchResponse switchController(const std::string & controllerName);

  /**
   * @brief Get a vector of all available controller names
   * @return vector of the available controller names
   */
  std::vector<std::string> getAvailableControllerNames();

  /**
   * @brief Get the current controller name
   * @return name of the currently active controller
   */
  std::string getActiveControllerName();

  /**
   * @brief Get isRealRobot
   * @return true if real robot
   */
  bool isRealRobot() const;

  /**
   * @brief Set isRealRobot
   */
  void setIsRealRobot(bool isRealRobot);

  virtual void reactOnEmergencyStop(rocoma::EmergencyStopObserver::EmergencyStopType type) { };

  void addEmergencystopObserver(EmergencyStopObserver* observer) {
    emergencyStopObservers_.push_back(observer);
    return;
  }

 private:
  //! Controller timestep (equal for all controllers)
  double timeStep_;

  //! Flag to differ between simulation and real robot
  bool isRealRobot_;

  //! Unordered map of all available controllers (owned by the manager)
  std::unordered_map< std::string, ControllerPtr > controllers_;
  std::unordered_map< std::string, EmgcyControllerPtr > emergencyControllers_;

  //! Controller Pairs
  std::unordered_map< std::string, ControllerPtrPtrPair > controllerPairs_;
  ControllerPtrPtrPair activeControllerPair_;

  //! Emergency stop controller
  FailproofControllerPtr failproofController_;

  //! Current controller state
  ManagedControllerState activeControllerState_;

  //! Mutex of the active controller
  std::mutex activeControllerMutex_;

  //! List of emergency stop observers
  std::list<EmergencyStopObserver*> emergencyStopObservers_;

  //! Stopping controllers
  std::mutex stopThreadsMutex_;
  std::vector<StopControllerThread> stoppingThreads_;

};

} /* namespace rocoma */
