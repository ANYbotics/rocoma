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
 * @file     ControllerRos.hpp
 * @author   Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Dec, 2014
 * @brief
 */
#pragma once

// Roco
#include <roco/controllers/adaptees/EmergencyControllerAdapteeInterface.hpp>
#include <roco/controllers/adapters/EmergencyControllerAdapterInterface.hpp>

// Rocoma
#include <rocoma/controllers/ControllerAdapter.hpp>

// STL
#include <type_traits>
#include <assert.h>
#include <memory>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class EmergencyControllerAdapter: virtual public roco::EmergencyControllerAdapterInterface, public ControllerAdapter<Controller_, State_, Command_>
{
  static_assert(std::is_base_of<roco::EmergencyControllerAdapteeInterface, Controller_>::value, "[EmergencyControllerAdapter]: The Controller class does not implement the EmergencyControllerAdatpeeInterface.");

 public:
  //! Convenience typedefs
  using Base = ControllerAdapter<Controller_, State_, Command_>;
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //! Delete default constructor
  EmergencyControllerAdapter() { };

  //! Virtual destructor
  virtual ~EmergencyControllerAdapter() { }

  //! Implement adapter
  virtual bool initializeControllerFast(double dt) {
    // Check if controller was created
    if (!this->isCreated()) {
      MELO_WARN_STREAM("[EmergencyControllerAdapter]: Controller was not created!");
      return false;
    }

    try {
      // Update the state
      this->updateState(dt, false);

      // Initialize the controller
      if (!this->initializeControllerFast(dt)) {
        MELO_WARN_STREAM("[EmergencyControllerAdapter]: Controller could not be fast initialized!");
        return false;
      }

      // Update the command
      this->updateCommand(dt);

      // Set init flag
      this->isInitialized_ = true;

    }
    catch (std::exception& e) {
      MELO_WARN_STREAM("[EmergencyControllerAdapter]: Exception caught:\n" << e.what());
      this->isInitialized_ = false;
      return false;
    }

    // Set running flag
    this->isRunning_ = true;
    MELO_INFO_STREAM("[EmergencyControllerAdapter]: Initialized controller fast" << this->getName() << " successfully!");
    return true;

  }

};

} // namespace rocoma
