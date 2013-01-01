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
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Oct, 2014
 */
#include <ros/package.h>

#include <locomotion_controller/ControllerManager.hpp>
#include <locomotion_controller/ControllerRos.hpp>

#include <roco_assembly/controllers.hpp>

namespace locomotion_controller {

void add_locomotion_controllers(locomotion_controller::ControllerManager* manager,
                                quadruped_model::State& state,
                                boost::shared_mutex& mutexState;
                                quadruped_model::Command& command,
                                boost::shared_mutex& mutexCommand;
                                ros::NodeHandle& nodeHandle) {

  std::string quadrupedName;
  nodeHandle.param<std::string>("quadruped/name", quadrupedName, "starleth");

#ifdef USE_TASK_LOCODEMO
  auto controllerLocoDemo = new ControllerRos<loco_demo::LocoDemo>(state, command, mutexState, mutexCommand);
  controllerLocoDemo->setParameterPath(ros::package::getPath("loco_demo_parameters") + "/" + quadrupedName);
  controllerLocoDemo->setControllerManager(manager);
  controllerLocoDemo->setIsRealRobotFromManager(manager->isRealRobot());
  manager->addController(controllerLocoDemo);
#endif

#ifdef USE_TASK_LOCODEMO_ROS
  auto controllerLocoDemoRos = new ControllerRos<loco_demo_ros::LocoDemoRos>(state, command, mutexState, mutexCommand);
  controllerLocoDemoRos->setParameterPath(ros::package::getPath("loco_demo_parameters") + "/" + quadrupedName);
  controllerLocoDemoRos->setControllerManager(manager);
  controllerLocoDemoRos->setIsRealRobotFromManager(manager->isRealRobot());
  controllerLocoDemoRos->setNodeHandle(nodeHandle);
  manager->addController(controllerLocoDemoRos);
#endif

#ifdef USE_TASK_LOCOCRAWLING
   auto controllerCrawling = new ControllerRos<loco_crawling::Crawling>(state, command, mutexState, mutexCommand);
   controllerCrawling->setParameterPath(ros::package::getPath("loco_crawling_parameters"));
   controllerCrawling->setQuadrupedName(quadrupedName);
   controllerCrawling->setControllerManager(manager);
   controllerCrawling->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerCrawling);
#endif

#ifdef USE_TASK_LOCOJUMP
   auto controllerJump = new ControllerRos<loco_jump::LocoJump>(state, command, mutexState, mutexCommand);
   controllerJump->setControllerManager(manager);
   controllerJump->setIsRealRobotFromManager(manager->isRealRobot());
   controllerJump->setPackageRoot(ros::package::getPath("loco_jump"));
   manager->addController(controllerJump);
#endif

#ifdef USE_TASK_LOCOTRACKER
   auto controllerTracker = new ControllerRos<loco_tracker::LocoTracker>(state, command, mutexState, mutexCommand);
   controllerTracker->setParameterPath(ros::package::getPath("loco_tracker"));
   controllerTracker->setControllerManager(manager);
   controllerTracker->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerTracker);
#endif

#ifdef USE_TASK_LOCOCRAWLING_ROS
   auto controllerCrawlingRos = new ControllerRos<loco_crawling_ros::LocoCrawlingRos>(state, command, mutexState, mutexCommand);
   controllerCrawlingRos->setParameterPath(ros::package::getPath("loco_crawling_parameters"));
   controllerCrawlingRos->setQuadrupedName(quadrupedName);
   controllerCrawlingRos->setControllerManager(manager);
   controllerCrawlingRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerCrawlingRos->setNodeHandle(nodeHandle);
   manager->addController(controllerCrawlingRos);
#endif

#ifdef USE_TASK_LOCOFREEGAIT
   auto controllerFreeGait = new ControllerRos<loco_free_gait::FreeGait>(state, command, mutexState, mutexCommand);
   controllerFreeGait->setControllerPath(ros::package::getPath("loco_free_gait"));
   controllerFreeGait->setControllerManager(manager);
   controllerFreeGait->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerFreeGait);
#endif

#ifdef USE_TASK_LOCOFREEGAIT_ROS
   auto controllerFreeGaitRos = new ControllerRos<loco_free_gait_ros::FreeGaitRos>(state, command, mutexState, mutexCommand);
   controllerFreeGaitRos->setControllerPath(ros::package::getPath("loco_free_gait"));
   controllerFreeGaitRos->setControllerManager(manager);
   controllerFreeGaitRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerFreeGaitRos->setNodeHandle(nodeHandle);
   manager->addController(controllerFreeGaitRos);
#endif

#ifdef USE_TASK_LOCOJOINTTRAJECTORIES
   auto controllerJointTrajectories = new ControllerRos<loco_joint_trajectories::JointTrajectories>(state, command, mutexState, mutexCommand);
   controllerJointTrajectories->setControllerPath(ros::package::getPath("loco_joint_trajectories"));
   controllerJointTrajectories->setControllerManager(manager);
   controllerJointTrajectories->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerJointTrajectories);
#endif

#ifdef USE_TASK_LOCOJOINTTRAJECTORIES_ROS
   auto controllerJointTrajectoriesRos = new ControllerRos<loco_joint_trajectories_ros::JointTrajectoriesRos>(state, command, mutexState, mutexCommand);
   controllerJointTrajectoriesRos->setControllerPath(ros::package::getPath("loco_joint_trajectories"));
   controllerJointTrajectoriesRos->setControllerManager(manager);
   controllerJointTrajectoriesRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerJointTrajectoriesRos->setNodeHandle(nodeHandle);
   manager->addController(controllerJointTrajectoriesRos);
#endif

#ifdef USE_TASK_ROCOSEATESTSTEP_ROS
   auto controllerRocoSeaTestStepRos = new ControllerRos<roco_sea_test_ros::RocoSeaTestStepRos>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestStepRos->setControllerManager(manager);
   controllerRocoSeaTestStepRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerRocoSeaTestStepRos->setNodeHandle(nodeHandle);
   manager->addController(controllerRocoSeaTestStepRos);
#endif

#ifdef USE_TASK_ROCO_RECOVERY_STATICWALK
   auto controllerRocoRecoveryStaticWalk = new ControllerRos<roco_recovery_staticwalk::RocoRecoveryStaticWalk>(state, command, mutexState, mutexCommand);
   controllerRocoRecoveryStaticWalk->setControllerPath(ros::package::getPath("roco_recovery_staticwalk"));
   controllerRocoRecoveryStaticWalk->setControllerManager(manager);
   controllerRocoRecoveryStaticWalk->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerRocoRecoveryStaticWalk);
#endif

#ifdef USE_TASK_ROCO_RECOVERY_STATICWALK_ROS
   auto controllerRocoRecoveryStaticWalkRos = new ControllerRos<roco_recovery_staticwalk_ros::RocoRecoveryStaticWalkRos>(state, command, mutexState, mutexCommand);
   controllerRocoRecoveryStaticWalkRos->setControllerPath(ros::package::getPath("roco_recovery_staticwalk"));
   controllerRocoRecoveryStaticWalkRos->setControllerManager(manager);
   controllerRocoRecoveryStaticWalkRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerRocoRecoveryStaticWalkRos->setNodeHandle(nodeHandle);
   manager->addController(controllerRocoRecoveryStaticWalkRos);
#endif

#ifdef USE_TASK_ROCOSEATESTCHIRP
   auto controllerRocoSeaTestChirp = new ControllerRos<roco_sea_test_chirp::RocoSeaTestChirp>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestChirp->setControllerManager(manager);
   controllerRocoSeaTestChirp->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerRocoSeaTestChirp);
#endif


#ifdef USE_TASK_ROCOSEATEST_FRICTION_ID
   auto controllerRocoSeaTestFrictionId = new ControllerRos<roco_sea_test_friction_id::RocoSeaTestFrictionId>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestFrictionId->setControllerManager(manager);
   controllerRocoSeaTestFrictionId->setIsRealRobotFromManager(manager->isRealRobot());
    manager->addController(controllerRocoSeaTestFrictionId);
#endif

#ifdef USE_TASK_ROCOSEATEST_ANYMAL
   auto controllerRocoSeaTestAnymal = new ControllerRos<roco_sea_test_anymal::RocoSeaTestAnymal>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestAnymal->setControllerManager(manager);
   controllerRocoSeaTestAnymal->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerRocoSeaTestAnymal);
#endif

#ifdef USE_TASK_ROCOSEATEST_SWING
   auto controllerRocoSeaTestSwing = new ControllerRos<roco_sea_test_swing::RocoSeaTestSwing>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestSwing->setControllerManager(manager);
   controllerRocoSeaTestSwing->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerRocoSeaTestSwing);
#endif

#ifdef USE_TASK_ROCOSEATEST_SWING_INVERSEDYNAMICS
   auto controllerRocoSeaTestSwingInverseDynamics = new ControllerRos<roco_sea_test_swing::RocoSeaTestSwingInverseDynamics>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestSwingInverseDynamics->setControllerManager(manager);
   controllerRocoSeaTestSwingInverseDynamics->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerRocoSeaTestSwingInverseDynamics);
#endif

#ifdef USE_TASK_ROCOSEATEST_GRAVITY_COMPENSATION
   auto controllerRocoSeaTestGravityCompensation = new ControllerRos<roco_sea_test_swing::RocoSeaTestGravityCompensation>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestGravityCompensation->setControllerManager(manager);
   controllerRocoSeaTestGravityCompensation->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerRocoSeaTestGravityCompensation);
#endif

#ifdef USE_TASK_LOCOBOUND
   auto controllerLocoBound = new ControllerRos<loco_bound::LocoBound>(state, command, mutexState, mutexCommand);
   controllerLocoBound->setControllerManager(manager);
   controllerLocoBound->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerLocoBound);
#endif

#ifdef USE_TASK_LOCOEXAMPLE
   auto controllerLocoExample = new ControllerRos<loco_example::LocoExample>(state, command, mutexState, mutexCommand);
   controllerLocoExample->setControllerManager(manager);
   controllerLocoExample->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerLocoExample);
#endif

#ifdef USE_TASK_LOCOGUIDEDTEACHING
   auto controllerGuidedTeaching = new ControllerRos<loco_guided_teaching::LocoGuidedTeaching>(state, command, mutexState, mutexCommand);
   controllerGuidedTeaching->setControllerPath(ros::package::getPath("loco_guided_teaching"));
   controllerGuidedTeaching->setControllerManager(manager);
   controllerGuidedTeaching->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerGuidedTeaching);
#endif

#ifdef USE_TASK_LOCOMOPT
  auto controllerLocoMopt = new ControllerRos<loco_mopt::LocoMopt>(state, command, mutexState, mutexCommand);
  controllerLocoMopt->setParameterPath(ros::package::getPath("loco_mopt_parameters") + "/parameters/" + quadrupedName);
  controllerLocoMopt->setControllerManager(manager);
  controllerLocoMopt->setIsRealRobotFromManager(manager->isRealRobot());
  manager->addController(controllerLocoMopt);
#endif

#ifdef USE_TASK_LOCOMOPT_ROS
   auto controllerLocoMoptRos = new ControllerRos<loco_mopt_ros::LocoMoptRos>(state, command, mutexState, mutexCommand);
   controllerLocoMoptRos->setParameterPath(ros::package::getPath("loco_mopt_parameters"));
   controllerLocoMoptRos->setParameterPath(ros::package::getPath("loco_mopt_parameters") + "/parameters/" + quadrupedName);
   controllerLocoMoptRos->setControllerManager(manager);
   controllerLocoMoptRos->setIsRealRobotFromManager(manager->isRealRobot());
   controllerLocoMoptRos->setNodeHandle(nodeHandle);
   manager->addController(controllerLocoMoptRos);
#endif


#ifdef USE_TASK_ROCOSEATEST_SCHEDULE
   auto controllerRocoSeaTestSchedule = new ControllerRos<roco_sea_test_schedule::RocoSeaTestSchedule>(state, command, mutexState, mutexCommand);
   controllerRocoSeaTestSchedule->setControllerManager(manager);
   controllerRocoSeaTestSchedule->setIsRealRobotFromManager(manager->isRealRobot());
   manager->addController(controllerRocoSeaTestSchedule);
#endif

}

}
