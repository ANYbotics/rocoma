/*
 * Model.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: gech
 */

#ifndef MODEL_HPP_
#define MODEL_HPP_

#include "RobotModel.hpp"
#include "robotUtils/terrains/TerrainBase.hpp"

#include <ros/ros.h>
#include <starleth_msgs/RobotState.h>
#include <sensor_msgs/Joy.h>
#include <starleth_msgs/SeActuatorCommands.h>

#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/rotations/RotationDiffEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

namespace model {

class Model
{
 public:
  typedef kindr::rotations::eigen_impl::RotationQuaternionPD RotationQuaternion;
  typedef kindr::rotations::eigen_impl::EulerAnglesZyxPD EulerAnglesZyx;
  typedef kindr::rotations::eigen_impl::LocalAngularVelocityPD LocalAngularVelocity;
  typedef kindr::rotations::eigen_impl::AngleAxisPD AngleAxis;
  typedef kindr::phys_quant::eigen_impl::Position3D Position;
  typedef kindr::phys_quant::eigen_impl::Velocity3D LinearVelocity;
  typedef kindr::phys_quant::eigen_impl::VectorTypeless3D Vector;
 public:
  Model();
  virtual ~Model();
  void initialize(double dt);

  void setRobotModelParameters();
  void setRobotState(const starleth_msgs::RobotState::ConstPtr& robotState);
  void getSeActuatorCommands(starleth_msgs::SeActuatorCommandsPtr& actuatorCommands);
  void setJoystickCommands(const sensor_msgs::Joy::ConstPtr& msg);


  robotModel::RobotModel* getRobotModel();
  robotTerrain::TerrainBase* getTerrainModel();
 private:
  std::shared_ptr<robotModel::RobotModel> robotModel_;
  std::shared_ptr<robotTerrain::TerrainBase> terrain_;
};

} /* namespace model */

#endif /* MODEL_HPP_ */
