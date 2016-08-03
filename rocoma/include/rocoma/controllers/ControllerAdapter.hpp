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
#include "roco/model/CommandInterface.hpp"
#include "roco/model/StateInterface.hpp"
#include "roco/controllers/Controller.hpp"
#include "roco/controllers/adapters/ControllerAdapterInterface.hpp"

// Rocoma
#include "rocoma/controllers/ControllerExtensionImplementation.hpp"

// STL
#include <exception>
#include <type_traits>
#include <assert.h>
#include <memory>
#include <atomic>


namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class ControllerAdapter: virtual public roco::ControllerAdapterInterface, public ControllerExtensionImplementation<Controller_, State_, Command_>
{
 public:
  //! Convenience typedefs
  using Base = ControllerExtensionImplementation<Controller_, State_, Command_>;
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //! Delete default constructor
  ControllerAdapter(): isStopping_(false), isInitializing_(false) { }

  //! Virtual destructor
  virtual ~ControllerAdapter() { }

  //! Implementation of the adapter interface (roco::ControllerAdapterInterface)
  virtual bool createController(double dt);
  virtual bool initializeController(double dt);
  virtual bool resetController(double dt);
  virtual bool advanceController(double dt);
  virtual bool cleanupController();
  virtual bool stopController();
  virtual bool preStopController();

  //! Set isRealRobot is only allowed on the adapter
  virtual void setIsRealRobot(bool isRealRobot)         { this->isRealRobot_ = isRealRobot; }

  //! Controller name adapter
  virtual const std::string& getControllerName() const  { return this->getName(); }

  //! Indicates if the controller is initialized
  virtual bool isControllerInitialized() const          { return this->isInitialized(); }
  //! Indicates if the controller is initializing at the moment
  virtual bool isInitializing() const { return isInitializing_; }
  //! Indicates if the controller is stopping at the moment
  virtual bool isStopping() const { return isStopping_; }

 protected:
  bool updateState(double dt, bool checkState = true);
  bool updateCommand(double dt);

  std::atomic<bool> isStopping_;
  std::atomic<bool> isInitializing_;


};

} // namespace rocoma


#include <rocoma/controllers/ControllerAdapter.tpp>
