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

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
roco::WorkerHandle ControllerExtensionImplementation<Controller_, State_, Command_>::addWorker(const roco::WorkerOptions&  options) {
  workers_.emplace(options.name_, WorkerWrapper());
  auto& wrapper = workers_[options.name_];
  wrapper.options_ = options;

  nodewrap::WorkerOptions workerOptions;
  workerOptions.autostart = options.autostart_;
  workerOptions.frequency = options.frequency_;
  workerOptions.priority = options.priority_;
  workerOptions.synchronous = options.synchronous_;
  workerOptions.callback = boost::bind(&WorkerWrapper::workerCallback, wrapper, _1);

  wrapper.worker_ = controllerManager_->getLocomotionController()->addWrappedWorker(options.name_, workerOptions);

  roco::WorkerHandle workerHandle;
  workerHandle.name_ = options.name_;

  return workerHandle;
}

template<typename Controller_, typename State_, typename Command_>
roco::WorkerHandle ControllerExtensionImplementation<Controller_, State_, Command_>::addWorker(roco::Worker& worker) {

  auto& options  = worker.options_;
  workers_.emplace(options.name_, WorkerWrapper());
  auto& wrapper = workers_[options.name_];

  wrapper.options_ = options;

  nodewrap::WorkerOptions workerOptions;
  workerOptions.autostart =  options.autostart_;
  workerOptions.frequency =  options.frequency_;
  workerOptions.priority =  options.priority_;
  workerOptions.synchronous =  options.synchronous_;
  workerOptions.callback = boost::bind(&WorkerWrapper::workerCallback, wrapper, _1);
  wrapper.worker_ = controllerManager_->getLocomotionController()->addWrappedWorker(options.name_, workerOptions);

  worker.workerStartCallback_ = boost::bind(&RobotController<Controller_,State_, Command_>::startWorker, this, _1);
  worker.workerCancelCallback_ = boost::bind(&RobotController<Controller_,State_, Command_>::cancelWorker, this, _1, _2 );

  worker.handle_.name_ = options.name_;
  return worker.handle_;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerExtensionImplementation<Controller_, State_, Command_>::startWorker(const roco::WorkerHandle& workerHandle) {
  MELO_INFO_STREAM("ControllerRos::startWorker: start " << workerHandle.name_);
  workers_[workerHandle.name_].worker_.start();
  MELO_INFO_STREAM("ControllerRos::startWorker started " << workerHandle.name_);
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerExtensionImplementation<Controller_, State_, Command_>::cancelWorker(const roco::WorkerHandle& workerHandle, bool block) {
  MELO_INFO_STREAM("ControllerRos::cancelWorker: cancel  " << workerHandle.name_);
  workers_[workerHandle.name_].worker_.cancel(block);
  return true;
}

} /** namespace rocoma */
