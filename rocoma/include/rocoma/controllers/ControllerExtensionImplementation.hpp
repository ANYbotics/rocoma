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
 * @file     ControllerExtensionImplementation.hpp
 * @author   Christian Gehring, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#pragma once

// rocoma
#include <rocoma/controllers/ControllerImplementation.hpp>

// roco
#include <roco/time/Time.hpp>
#include <roco/workers/Worker.hpp>
#include <roco/workers/WorkerHandle.hpp>
#include <roco/workers/WorkerOptions.hpp>

// Boost
#include <boost/thread.hpp>

// AnyWorker
#include <any_worker/WorkerManager.hpp>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class ControllerExtensionImplementation: public ControllerImplementation<Controller_, State_, Command_> {

 public:
  //! Convenience typedefs
  using Base = ControllerImplementation<Controller_, State_, Command_>;
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  ControllerExtensionImplementation() = delete;

  ControllerExtensionImplementation(State& state,
                                    Command& command,
                                    boost::shared_mutex& mutexState,
                                    boost::shared_mutex& mutexCommand);

  virtual ~ControllerExtensionImplementation();

  virtual bool isRealRobot() const                      { return isRealRobot_; }

  virtual const roco::time::Time& getTime() const       { return static_cast<const roco::time::Time&>(time_); }
  virtual void setTime(const roco::time::Time& time)    { time_ = time;}

  virtual bool isCheckingCommand() const                { return isCheckingCommand_; }
  virtual void setIsCheckingCommand(bool isChecking)    { isCheckingCommand_ = isChecking; }

  virtual bool isCheckingState() const                  { return isCheckingState_; }
  virtual void setIsCheckingState(bool isChecking)      { isCheckingState_ = isChecking; }

  virtual roco::WorkerHandle addWorker(const roco::WorkerOptions& options);
  virtual roco::WorkerHandle addWorker(roco::Worker& worker);
  virtual bool startWorker(const roco::WorkerHandle& workerHandle);
  virtual bool cancelWorker(const roco::WorkerHandle& workerHandle, bool block = false);

 private:
  //! Indicates if the real robot is controller or only a simulated version.
  boost::atomic<bool> isRealRobot_;
  //! Indicates if command is checked for limits
  boost::atomic<bool> isCheckingCommand_;
  //! Indicates if state is checked for validity
  boost::atomic<bool> isCheckingState_;
  //! Time
  roco::time::TimeStd time_;
  //! Worker Manager
  any_worker::WorkerManager workerManager_;

};

} /** namespace rocoma */

#include <rocoma/controllers/ControllerExtensionImplementation.tpp>
