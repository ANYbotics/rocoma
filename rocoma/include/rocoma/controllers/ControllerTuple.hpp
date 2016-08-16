/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Gabriel Hottiger
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
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich
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
 * @file	ControllerTuple.hpp
 * @author	Gabriel Hottiger
 * @date	Aug 15, 2016
 */

#pragma once

// roco
#include "roco/controllers/Controller.hpp"
#include "roco/controllers/ControllerRos.hpp"

// STL
#include <algorithm>
#include <exception>

namespace rocoma {

template <typename Base_, typename State_, typename Command_, typename... Controllers_>
class ControllerTupleBase: virtual public Base_, public Controllers_ ...
{
 public:

  // This allows returning directly when one of the controllers returns false
  class ControllerTupleException: public std::exception
  {
    virtual const char* what() const throw()
    {
      return "Controller tuple exception!";
    }
  } myex;

  ControllerTupleBase():
    Base_(),
    Controllers_()...
  {
  }

  virtual ~ControllerTupleBase() { }

 protected:
  //! Roco implementation
  virtual bool create(double dt)
  {
    try {
      std::initializer_list<bool> list = {(Controllers_::create(dt)?true:throw(myex))...};
    }
    catch(ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  virtual bool initialize(double dt)
  {
    try {
      std::initializer_list<bool> list = {(Controllers_::initialize(dt)?true:throw(myex))...};
    }
    catch(ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  virtual bool reset(double dt)
  {
    try {
      std::initializer_list<bool> list = {(Controllers_::reset(dt)?true:throw(myex))...};
    }
    catch(ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  virtual bool advance(double dt)
  {
    try {
      std::initializer_list<bool> list = {(Controllers_::advance(dt)?true:throw(myex))...};
    }
    catch(ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  virtual bool preStop()
  {
    try {
      std::initializer_list<bool> list = {(Controllers_::preStop()?true:throw(myex))...};
    }
    catch(ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  virtual bool stop()
  {
    try {
      std::initializer_list<bool> list = {(Controllers_::stop()?true:throw(myex))...};
    }
    catch(ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  virtual bool cleanup()
  {
    try {
      std::initializer_list<bool> list = {(Controllers_::cleanup()?true:throw(myex))...};
    }
    catch(ControllerTupleException& e) {
      return false;
    }
    return true;
  }

};

template <typename State_, typename Command_, typename... Controllers_>
using ControllerTuple = ControllerTupleBase< roco::Controller<State_, Command_>, State_, Command_, Controllers_... >;

template <typename State_, typename Command_, typename... Controllers_>
using ControllerTupleRos = ControllerTupleBase< roco::ControllerRos<State_, Command_>, State_, Command_, Controllers_... >;

} /* namespace rocoma */
