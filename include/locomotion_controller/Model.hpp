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

#pragma once

#include "robotUtils/terrains/TerrainBase.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <quadruped_msgs/QuadrupedState.h>
#include <sensor_msgs/Joy.h>
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>
#include <series_elastic_actuator_msgs/SeActuatorCommand.h>
#include <series_elastic_actuator_msgs/SeActuatorState.h>

#include <quadruped_model/QuadrupedModel.hpp>
#include <quadruped_model/common/State.hpp>
#include <quadruped_model/common/Command.hpp>

#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/rotations/RotationDiffEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

namespace model {

class Model
{
 public:
  static constexpr int numberOfJoints_ = 12;
  typedef kindr::rotations::eigen_impl::RotationQuaternionPD RotationQuaternion;
  typedef kindr::rotations::eigen_impl::RotationMatrixPD     RotationMatrix;
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
  void initializeForController(double dt,
                               bool isRealRobot,
                               const std::string& urdfDescription,
                               const quadruped_model::Quadrupeds& quadruped);
  void initializeForStateEstimator(double dt,
                                   bool isRealRobot,
                                   const std::string& pathToUrdfFile,
                                   const quadruped_model::Quadrupeds& quadruped);
  void reinitialize(double dt);
  void addVariablesToLog();

//  void setRobotModelParameters();
  void setQuadrupedState(const quadruped_msgs::QuadrupedState::ConstPtr& quadrupedState);
  void setQuadrupedState(const sensor_msgs::ImuPtr& imu,
                     const sensor_msgs::JointStatePtr& jointState,
                     const geometry_msgs::WrenchStampedPtr& contactForceLf,
                     const geometry_msgs::WrenchStampedPtr& contactForceRf,
                     const geometry_msgs::WrenchStampedPtr& contactForceLh,
                     const geometry_msgs::WrenchStampedPtr& contactForceRh,
                     const Eigen::Vector4i& contactFlags);
  void initializeQuadrupedState(quadruped_msgs::QuadrupedStatePtr& quadrupedState) const;
  void initializeJointState(sensor_msgs::JointState& jointState) const;

  void getQuadrupedState(quadruped_msgs::QuadrupedStatePtr& quadrupedState);
  void getSeActuatorCommands(series_elastic_actuator_msgs::SeActuatorCommandsPtr& actuatorCommands);
  void getPose(geometry_msgs::PoseWithCovarianceStampedPtr& pose);
  void getTwist(geometry_msgs::TwistWithCovarianceStampedPtr& pose);

  void setJoystickCommands(const sensor_msgs::Joy::ConstPtr& msg);
  void setCommandVelocity(const geometry_msgs::Twist& msg);
  void setMocapData(const geometry_msgs::TransformStamped::ConstPtr& msg);

  void setSeActuatorReadings(const series_elastic_actuator_msgs::SeActuatorReadings::ConstPtr& readings);
  void setSeActuatorState(const int iJoint, const series_elastic_actuator_msgs::SeActuatorState& state);
  void setSeActuatorCommanded(const int iJoint, const series_elastic_actuator_msgs::SeActuatorCommand& commanded);
  quadruped_model::QuadrupedModel* getQuadrupedModel();
  robotTerrain::TerrainBase* getTerrainModel();
  quadruped_model::State& getState();
  quadruped_model::Command& getCommand();

  const quadruped_model::State& getState() const;
  const quadruped_model::Command& getCommand() const;

 private:
  ros::Time updateStamp_;
  std::shared_ptr<quadruped_model::QuadrupedModel> quadrupedModel_;
  std::shared_ptr<robotTerrain::TerrainBase> terrain_;
  quadruped_model::State state_;
  quadruped_model::Command command_;
};

} /* namespace model */

