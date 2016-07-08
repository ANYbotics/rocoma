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
#include <roco/model/CommandInterface.hpp>
#include <roco/model/StateInterface.hpp>
#include <roco/controllers/adapters/ControllerAdapterInterface.hpp>

// Rocoma
#include <rocoma/ControllerExtensionImplementation.hpp>

// STL
#include <exception>
#include <type_traits>
#include <assert.h>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class ControllerAdapter: public roco::ControllerAdapterInterface, public ControllerExtensionImplementation<Controller_, State_, Command_>
{
  //! Check if template parameters implement the required interfaces
  static_assert(std::is_base_of<roco::StateInterface, State_>::value, "[ControllerRos]: The State class does not implement roco::StateInterface!" );
  static_assert(std::is_base_of<roco::CommandInterface, Command_>::value, "[ControllerRos]: The Command class does not implement roco::CommandInterface!" );
  static_assert(std::is_base_of<roco::Controller<State_, Command_>, Controller_>::value, "[ControllerRos]: The Controller class does not inherit from roco::Controllers<State_, Command_>!" );

 public:
  //! Convenience typedefs
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //! Delete default constructor
  ControllerAdapter() = delete;

  /**
   * @brief Constructor for state and command
   * @param state         robot state container class
   * @param command       actutator command containter class
   * @param mutexState    mutex for state class
   * @param mutexCommand  mutex for command class
   */
  ControllerAdapter(State& state,
                    Command& command,
                    boost::shared_mutex& mutexState,
                    boost::shared_mutex& mutexCommand);

  //! Virtual destructor
  virtual ~ControllerAdapter();

  //! Implementation of the adapter interface (roco::ControllerAdapterInterface)
  virtual bool createController(double dt);
  virtual bool initializeController(double dt);
  virtual bool resetController(double dt);
  virtual bool advanceController(double dt);
  virtual bool changeController();
  virtual bool cleanupController();
  virtual bool stopController();
  virtual bool preStopController();


  //! Set isRealRobot is only allowed on the adapter
  virtual void setIsRealRobot(bool isRealRobot);

 protected:
  bool updateState(double dt, bool checkState=true);
  bool updateCommand(double dt, bool forceSendingControlModes);

};

} // namespace rocoma


#include <rocoma/ControllerAdapter.tpp>
