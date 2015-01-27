/*
 * Copyright (c) 2014, Christian Gehring
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL  Christian Gehring BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/*!
 * @file    locomotion_controllers.cpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */
#include <locomotion_controller/ControllerManager.hpp>
#include <locomotion_controller/ControllerRos.hpp>

#include <roco_assembly/controllers.hpp>

namespace locomotion_controller {

void add_locomotion_controllers(locomotion_controller::ControllerManager* manager, robotModel::State& state, robotModel::Command& command) {

#ifdef USE_TASK_LOCODEMO
  auto controllerLocoDemo = new ControllerRos<robotTask::LocoDemo>(state, command);
  controllerLocoDemo->setControllerManager(manager);
  controllerLocoDemo->setIsRealRobotFromManager(manager->isRealRobot());
  manager->addController(controllerLocoDemo);
#endif

#ifdef USE_TASK_LOCOCRAWLING
   auto controllerCrawling = new ControllerRos<robotTask::Crawling>(state, command);
   controllerCrawling->setControllerManager(manager);
   controllerCrawling->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerCrawling);
#endif

#ifdef USE_TASK_LOCOFREEGAIT
   auto controllerFreeGait = new ControllerRos<robotTask::FreeGait>(state, command);
   controllerFreeGait->setControllerManager(manager);
   controllerFreeGait->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerFreeGait);
#endif

}

}
