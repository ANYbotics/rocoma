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
 * @file     ControllerImplementation.hpp
 * @author   Christian Gehring, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#pragma once

// Boost
#include <boost/thread.hpp>

// STL
#include <memory>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class ControllerImplementation: public Controller_ {

 public:
  //! Convenience typedefs
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //  ControllerImplementation() = delete;
  ControllerImplementation():
    Controller(),
    state_(),
    command_(),
    mutexState_(),
    mutexCommand_()
 {

 }

  ControllerImplementation(std::shared_ptr<State> state,
                           std::shared_ptr<Command> command,
                           std::shared_ptr<boost::shared_mutex> mutexState,
                           std::shared_ptr<boost::shared_mutex> mutexCommand):
     Controller(),
     state_(state),
     command_(command),
     mutexState_(mutexState),
     mutexCommand_(mutexCommand)
  {

  }

  virtual ~ControllerImplementation()
  {

  }

  virtual void setStateAndCommand(std::shared_ptr<State> state,
                                  std::shared_ptr<boost::shared_mutex> mutexState,
                                  std::shared_ptr<Command> command,
                                  std::shared_ptr<boost::shared_mutex> mutexCommand) {
    state_ = state;
    mutexState_ = mutexState;
    command_ = command;
    mutexCommand_ = mutexCommand;
  }

  virtual const State& getState() const           { return *state_; }
  virtual boost::shared_mutex& getStateMutex()    { return *mutexState_; }

  virtual const Command& getCommand() const       { return *command_; }
  virtual Command& getCommand()                   { return *command_; }
  virtual boost::shared_mutex& getCommandMutex()  { return *mutexCommand_; }

 private:
  //! Robot state container
  std::shared_ptr<State> state_;
  //! Robot state container mutex
  std::shared_ptr<boost::shared_mutex> mutexState_;
  //! Actuator command container
  std::shared_ptr<Command> command_;
  //! Actuator command container mutex
  std::shared_ptr<boost::shared_mutex> mutexCommand_;

};

} /** namespace rocoma */
