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
#include <ros/package.h>

#include <locomotion_controller/ControllerManager.hpp>
#include <locomotion_controller/ControllerRos.hpp>

#include <ros/package.h>

#include <roco_assembly/controllers.hpp>

namespace locomotion_controller {

void add_locomotion_controllers(locomotion_controller::ControllerManager* manager, robot_model::State& state, robot_model::Command& command, ros::NodeHandle& nodeHandle) {

#ifdef USE_TASK_LOCODEMO
  auto controllerLocoDemo = new ControllerRos<loco_demo::LocoDemo>(state, command);
  controllerLocoDemo->setParameterPath(ros::package::getPath("loco_demo"));
  controllerLocoDemo->setControllerManager(manager);
  controllerLocoDemo->setIsRealRobotFromManager(manager->isRealRobot());
  manager->addController(controllerLocoDemo);
#endif

#ifdef USE_TASK_LOCODEMO_ROS
  auto controllerLocoDemoRos = new ControllerRos<loco_demo_ros::LocoDemoRos>(state, command);
  controllerLocoDemoRos->setParameterPath(ros::package::getPath("loco_demo"));
  controllerLocoDemoRos->setControllerManager(manager);
  controllerLocoDemoRos->setIsRealRobotFromManager(manager->isRealRobot());
  manager->addController(controllerLocoDemoRos);
#endif

#ifdef USE_TASK_LOCOCRAWLING
   auto controllerCrawling = new ControllerRos<loco_crawling::Crawling>(state, command);
   controllerCrawling->setParameterPath(ros::package::getPath("loco_crawling"));
   controllerCrawling->setControllerManager(manager);
   controllerCrawling->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerCrawling);
#endif

#ifdef USE_TASK_LOCOJUMP
   auto controllerJump = new ControllerRos<loco_jump::LocoJump>(state, command);
   controllerJump->setControllerManager(manager);
   controllerJump->setIsRealRobotFromManager(manager->isRealRobot());
   controllerJump->setPackageRoot(ros::package::getPath("loco_jump"));
   manager->addController(controllerJump);
#endif

#ifdef USE_TASK_LOCOCRAWLING_ROS
   auto controllerCrawlingRos = new ControllerRos<loco_crawling_ros::LocoCrawlingRos>(state, command);
   controllerCrawlingRos->setParameterPath(ros::package::getPath("loco_crawling"));
   controllerCrawlingRos->setControllerManager(manager);
   controllerCrawlingRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerCrawlingRos->setNodeHandle(nodeHandle);
   manager->addController(controllerCrawlingRos);
#endif

#ifdef USE_TASK_LOCOFREEGAIT
   auto controllerFreeGait = new ControllerRos<loco_free_gait::FreeGait>(state, command);
   controllerFreeGait->setControllerManager(manager);
   controllerFreeGait->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerFreeGait);
#endif

#ifdef USE_TASK_LOCOFREEGAIT_ROS
   auto controllerFreeGaitRos = new ControllerRos<loco_free_gait_ros::FreeGaitRos>(state, command);
   controllerFreeGaitRos->setControllerManager(manager);
   controllerFreeGaitRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerFreeGaitRos->setNodeHandle(nodeHandle);
   manager->addController(controllerFreeGaitRos);
#endif

#ifdef USE_TASK_ROCOSEATESTSTEP_ROS
   auto controllerRocoSeaTestStepRos = new ControllerRos<roco_sea_test_ros::RocoSeaTestStepRos>(state, command);
   controllerRocoSeaTestStepRos->setControllerManager(manager);
   controllerRocoSeaTestStepRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerRocoSeaTestStepRos->setNodeHandle(nodeHandle);
   manager->addController(controllerRocoSeaTestStepRos);
#endif



}

}
