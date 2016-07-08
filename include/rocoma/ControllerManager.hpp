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

// Controller Interface
#include <roco/controllers/adapters/ControllerAdapterInterface.hpp>

// Boost
#include <boost/ptr_container/ptr_unordered_map.hpp>

// STL
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>

namespace rocoma {

class ControllerManager
{
 public:
  //! Convenience typedefs
  using Controller = roco::ControllerAdapterInterface;
  using ControllerPtr = roco::ControllerAdapterInterface*;

  //! Enumeration for switch controller feedback
  enum class SwitchResponse : int {
      NOTFOUND = -2,
      ERROR    = -1,
      RUNNING  =  0,
      SWITCHED =  1
  };

 public:
  //! Delete default constructor
  ControllerManager() = delete;

  /**
   * @brief Constructs a controller manager
   * @param emergencyStopController   Controller that is active during an emergency stop
   * @param fallbackController        Controller that is executed after the active controller (default: none(nullptr))
   * @return the constructed ControllerManager
   */
  ControllerManager(std::unique_ptr<Controller> emergencyStopController,
                    std::unique_ptr<Controller> fallbackController = std::unique_ptr<Controller>());

  //! Destructor
  virtual ~ControllerManager();

  /**
   * @brief Add a controller to the controllers map
   * @param controller    Pointer to the controller (unique ptr -> ownership transfer)
   * @return true, if controller was created and added successfully
   */
  bool addController(std::unique_ptr<Controller> controller);

  /**
   * @brief Advance the currently active controller
   * @return true, if controller was advanced successfully
   */
  bool updateController();

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

 private:
  //! Controller timestep (equal for all controllers)
  double timeStep_;

  //! Flag to differ between simulation and real robot
  bool isRealRobot_;

  //! Unordered map of all available controllers (owned by the manager)
  std::unordered_map< std::string, std::unique_ptr<Controller> > controllers_;

  //! Emergency stop controller
  std::unique_ptr<Controller> emergencyStopController_;

  //! Fallback controller ( executed after the currently active controller )
  std::unique_ptr<Controller> fallbackController_;

  //! Pointer to the active controller pointer
  std::unique_ptr<Controller>* activeController_;

  //! Mutex of the active controller
  std::recursive_mutex activeControllerMutex_;

};

} /* namespace rocoma */
