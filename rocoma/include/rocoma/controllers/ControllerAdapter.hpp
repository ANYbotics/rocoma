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


namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class ControllerAdapter: virtual public roco::ControllerAdapterInterface, public ControllerExtensionImplementation<Controller_, State_, Command_>
{
  //! Check if template parameters implement the required interfaces
  static_assert(std::is_base_of<roco::StateInterface, State_>::value, "[ControllerAdapter]: The State class does not implement roco::StateInterface!" );
  static_assert(std::is_base_of<roco::CommandInterface, Command_>::value, "[ControllerAdapter]: The Command class does not implement roco::CommandInterface!" );
  static_assert(std::is_base_of<roco::Controller<State_, Command_>, Controller_>::value, "[ControllerAdapter]: The Controller class does not inherit from roco::Controller<State_, Command_>!" );

 public:
  //! Convenience typedefs
  using Base = ControllerExtensionImplementation<Controller_, State_, Command_>;
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //! Delete default constructor
//  ControllerAdapter() = delete;
  ControllerAdapter():Base() { }

  /**
   * @brief Constructor for state and command
   * @param state         robot state container class
   * @param command       actutator command container class
   * @param mutexState    mutex for state class
   * @param mutexCommand  mutex for command class
   */
  ControllerAdapter(std::shared_ptr<State> state,
                    std::shared_ptr<Command> command,
                    std::shared_ptr<boost::shared_mutex> mutexState,
                    std::shared_ptr<boost::shared_mutex> mutexCommand,
                    std::shared_ptr<any_worker::WorkerManager> workerManager);

  //! Virtual destructor
  virtual ~ControllerAdapter();

  //! Implementation of the adapter interface (roco::ControllerAdapterInterface)
  virtual bool createController(double dt);
  virtual bool initializeController(double dt);
  virtual bool resetController(double dt);
  virtual bool advanceController(double dt);
  virtual bool cleanupController();
  virtual bool stopController();
  virtual bool preStopController();

  //! Set isRealRobot is only allowed on the adapter
  virtual void setIsRealRobot(bool isRealRobot);

  //! Controller name adapter
  virtual const std::string& getControllerName() const { return this->getName(); }

  //! Indicates if the controller is initialized
  virtual bool isControllerInitialized() const { return this->isInitialized(); }

 protected:
  bool updateState(double dt, bool checkState = true);
  bool updateCommand(double dt);

};

} // namespace rocoma


#include <rocoma/controllers/ControllerAdapter.tpp>
