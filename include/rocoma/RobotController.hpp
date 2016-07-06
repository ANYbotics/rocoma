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

// roco
#include <roco/time/Time.hpp>
#include <roco/time/TimeStd.hpp>

#include <roco/model/CommandInterface.hpp>
#include <roco/model/StateInterface.hpp>
#include <roco/controllers/Controller.hpp>
#include <roco/controllers/ControllerAdapterInterface.hpp>

//#include <signal_logger/logger.hpp>

#include <boost/thread.hpp>
#include <rocoma/ControllerManager.hpp>
#include <rocoma/WorkerWrapper.hpp>
#include <iostream>
#include <exception>      // std::exception
#include <type_traits>
#include <assert.h>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class RobotController:  public roco::ControllerAdapterInterface//, public Controller_
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
  RobotController() = delete;

  /**
   * @brief Constructor for state and command
   * @param state         robot state container class
   * @param command       actutator command containter class
   * @param mutexState    mutex for state class
   * @param mutexCommand  mutex for command class
   */
  RobotController(State& state,
                  Command& command,
                  boost::shared_mutex& mutexState,
                  boost::shared_mutex& mutexCommand);

  //! Virtual destructor
  virtual ~RobotController();

  /** Implementation of the adaptee interface (roco::ControllerAdapteeInterface)
   */
  virtual const roco::time::Time& getTime() const;
  virtual void setTime(const roco::time::Time& time);

  virtual bool isCheckingCommand() const;
  virtual void setIsCheckingCommand(bool isChecking);

  virtual bool isCheckingState() const;
  virtual void setIsCheckingState(bool isChecking);

  virtual roco::WorkerHandle addWorker(const roco::WorkerOptions& options);
  virtual roco::WorkerHandle addWorker(roco::Worker& worker);
  virtual bool startWorker(const roco::WorkerHandle& workerHandle);
  virtual bool cancelWorker(const roco::WorkerHandle& workerHandle, bool block=false);

  /** Implementation of the controller interface (roco::Controller)
   */
  virtual const State& getState() const;
  virtual boost::shared_mutex& getStateMutex();

  virtual const Command& getCommand() const;
  virtual Command& getCommand();
  virtual boost::shared_mutex& getCommandMutex();

  /** Implementation of the adapter interface (roco::ControllerAdapterInterface)
   *  All functions of the interface that are not inherited from (roco::ControllerInterface)
   *  must be implemented here.
   */
  virtual bool createController(double dt);
  virtual bool initializeController(double dt);
  virtual bool resetController(double dt);
  virtual bool advanceController(double dt);
  virtual bool changeController();
  virtual bool cleanupController();
  virtual bool stopController();
  virtual bool preStopController();
  virtual void swapOut();

  /** Implementation of the common interface (roco::ControllerInterface)
   *  The remaining functions are already implemented in (roco::Controller)
   */
  virtual bool isRealRobot() const;

  /** Additonal functionality introduced only by the adapter
   *
   */
  virtual void setIsRealRobot(bool isRealRobot);
  void setControllerManager(ControllerManager* controllerManager);
  virtual void emergencyStop();
  virtual void startWorkers();
  virtual void cancelWorkers();

 protected:
  bool updateState(double dt, bool checkState=true);
  bool updateCommand(double dt, bool forceSendingControlModes);
  void sendEmergencyCommand();

 private:
  //! Indicates if the real robot is controller or only a simulated version.
  boost::atomic<bool> isRealRobot_;
  //! Indicates if command is checked for limits
  boost::atomic<bool> isCheckingCommand_;
  //! Indicates if state is checked for validity
  boost::atomic<bool> isCheckingState_;
  //! Indicate if this controller is the emergency controller
  boost::atomic<bool> isEmergencyController_;
  //! Time
  roco::time::TimeStd time_;
  //! Robot state container
  State& state_;
  //! Robot state container mutex
  boost::shared_mutex& mutexState_;
  //! Actuator command container
  Command& command_;
  //! Actuator command container mutex
  boost::shared_mutex& mutexCommand_;
  //! Controller manager
  ControllerManager* controllerManager_;

  //! TODO Worker impl
  nodewrap::Worker signalLoggerWorker_;
  std::map<std::string, WorkerWrapper> workers_;
  //! Worker callbacks
  virtual bool signalLoggerWorker(const nodewrap::WorkerEvent& event);

};

} // namespace rocoma


#include <rocoma/RobotController.tpp>
