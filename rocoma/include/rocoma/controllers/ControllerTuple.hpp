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

// STL
#include <algorithm>

namespace rocoma {

template <typename State_, typename Command_, typename... Controllers_>
class ControllerTuple: virtual public roco::Controller<State_, Command_>, public Controllers_ ...
{
 public:
  ControllerTuple() { }
  virtual ~ControllerTuple() { }

  //! Roco implementation
  virtual bool create(double dt)
  {
    std::initializer_list<bool> list = {(Controllers_::create(dt))...};
    return std::find(list.begin(), list.end(), false) == list.end();
  }

  virtual bool initialize(double dt)
  {
    std::initializer_list<bool> list = {(Controllers_::initialize(dt))...};
    return std::find(list.begin(), list.end(), false) == list.end();
  }

  virtual bool reset(double dt)
  {
    std::initializer_list<bool> list = {(Controllers_::reset(dt))...};
    return std::find(list.begin(), list.end(), false) == list.end();
  }

  virtual bool advance(double dt)
  {
    std::initializer_list<bool> list = {(Controllers_::advance(dt))...};
    return std::find(list.begin(), list.end(), false) == list.end();
  }

  virtual bool preStop()
  {
    std::initializer_list<bool> list = {(Controllers_::preStop())...};
    return std::find(list.begin(), list.end(), false) == list.end();
  }

  virtual bool stop()
  {
    std::initializer_list<bool> list = {(Controllers_::stop())...};
    return std::find(list.begin(), list.end(), false) == list.end();
  }

  virtual bool cleanup()
  {
    std::initializer_list<bool> list = {(Controllers_::cleanup())...};
    return std::find(list.begin(), list.end(), false) == list.end();
  }

};

} /* namespace rocoma */
