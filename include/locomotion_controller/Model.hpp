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
 * @file    Model.hpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */

#ifndef LOCOMOTION_CONTROLLER_MODEL_HPP_
#define LOCOMOTION_CONTROLLER_MODEL_HPP_

#include "starlethModel/RobotModel.hpp"
#include "robotUtils/terrains/TerrainBase.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <starleth_msgs/RobotState.h>
#include <sensor_msgs/Joy.h>
#include <starleth_msgs/SeActuatorCommands.h>

#include <starlethModel/State.hpp>
#include <starlethModel/Command.hpp>


#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/rotations/RotationDiffEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

namespace model {

class Model
{
 public:
  static constexpr int numberOfJoints_ = 12;
  typedef kindr::rotations::eigen_impl::RotationQuaternionPD RotationQuaternion;
  typedef kindr::rotations::eigen_impl::EulerAnglesZyxPD EulerAnglesZyx;
  typedef kindr::rotations::eigen_impl::LocalAngularVelocityPD LocalAngularVelocity;
  typedef kindr::rotations::eigen_impl::AngleAxisPD AngleAxis;
  typedef kindr::phys_quant::eigen_impl::Position3D Position;
  typedef kindr::phys_quant::eigen_impl::Velocity3D LinearVelocity;
  typedef kindr::phys_quant::eigen_impl::VectorTypeless3D Vector;
  typedef kindr::phys_quant::eigen_impl::Force3D Force;

  typedef kindr::phys_quant::eigen_impl::Position<double, numberOfJoints_> JointPositions;
  typedef kindr::phys_quant::eigen_impl::Velocity<double, numberOfJoints_> JointVelocities;
  typedef kindr::phys_quant::eigen_impl::Torque<double, numberOfJoints_> JointTorques;
 public:
  Model();
  virtual ~Model();
  void initializeForController(double dt,bool isRealRobot);
  void initializeForStateEstimator(double dt, bool isRealRobot);
  void reinitialize(double dt);
  void addVariablesToLog();

  void setRobotModelParameters();
  void setRobotState(const starleth_msgs::RobotState::ConstPtr& robotState);
  void setRobotState(const sensor_msgs::ImuPtr& imu,
                     const sensor_msgs::JointStatePtr& jointState,
                     const geometry_msgs::WrenchStampedPtr& contactForceLf,
                     const geometry_msgs::WrenchStampedPtr& contactForceRf,
                     const geometry_msgs::WrenchStampedPtr& contactForceLh,
                     const geometry_msgs::WrenchStampedPtr& contactForceRh);
  void initializeRobotState(starleth_msgs::RobotStatePtr& robotState) const;
  void initializeJointState(sensor_msgs::JointState& jointState) const;

  void getRobotState(starleth_msgs::RobotStatePtr& robotState);
  void getSeActuatorCommands(starleth_msgs::SeActuatorCommandsPtr& actuatorCommands);
  void setJoystickCommands(const sensor_msgs::Joy::ConstPtr& msg);


  robotModel::RobotModel* getRobotModel();
  robotTerrain::TerrainBase* getTerrainModel();
  robotModel::State& getState();
  robotModel::Command& getCommand();

  const robotModel::State& getState() const;
  const robotModel::Command& getCommand() const;

  void setContactForceThreshold(double value);
 private:
  ros::Time updateStamp_;
  std::shared_ptr<robotModel::RobotModel> robotModel_;
  std::shared_ptr<robotTerrain::TerrainBase> terrain_;
  robotModel::State state_;
  robotModel::Command command_;
  double contactForceThreshold_;
};

} /* namespace model */

#endif /* MODEL_HPP_ */
