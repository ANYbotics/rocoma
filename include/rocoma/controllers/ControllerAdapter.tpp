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
 * @file     ControllerRos.tpp
 * @author   Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Dec, 2014
 * @brief
 */

// Message logger
#include <message_logger/message_logger.hpp>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
ControllerAdapter<Controller_, State_, Command_>::ControllerAdapter(State& state,
                                                                    Command& command,
                                                                    boost::shared_mutex& mutexState,
                                                                    boost::shared_mutex& mutexCommand):
    Base(state, command, mutexState, mutexCommand)
{


}

template<typename Controller_, typename State_, typename Command_>
ControllerAdapter<Controller_, State_, Command_>::~ControllerAdapter()
{

}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::createController(double dt)
{
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::initializeController(double dt)
{
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::resetController(double dt)
{
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::changeController()
{
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::cleanupController()
{
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::stopController()
{
  return true;
}


template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::preStopController()
{
  return true;
}

template<typename Controller_, typename State_, typename Command_>
void ControllerAdapter<Controller_,State_, Command_>::setIsRealRobot(bool isRealRobot)
{
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateState(double dt, bool checkState)
{
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateCommand(double dt, bool forceSendingControlModes)
{
  return true;
}

}  // namespace rocoma
