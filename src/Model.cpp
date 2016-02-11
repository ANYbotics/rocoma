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
 * @file    Model.cpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Oct, 2014
 */

#include "locomotion_controller/Model.hpp"
#include "robotUtils/terrains/TerrainPlane.hpp"

#include <signal_logger/logger.hpp>
#include <series_elastic_actuator_ros/ConvertRosMessages.hpp>

// quadruped state and command helpers
#include <quadruped_model/common/quadruped_model_common.hpp>
#include <quadruped_model/robots/quadrupeds.hpp>
#include <quadruped_model/robots/anymal.hpp>
#include <quadruped_model/robots/starleth.hpp>

#include <quadruped_assembly/quadrupeds.hpp>

#include <roco/log/log_messages.hpp>

namespace model {

Model::Model():
  updateStamp_(0.0),
  quadrupedModel_(),
  terrain_(),
  state_(),
  command_()
{


}

Model::~Model()
{

}


quadruped_model::QuadrupedModel* Model::getQuadrupedModel() {
  return quadrupedModel_.get();
}
robotTerrain::TerrainBase* Model::getTerrainModel() {
  return terrain_.get();
}

quadruped_model::State& Model::getState() {
  return state_;
}

const quadruped_model::State& Model::getState() const {
  return state_;
}

boost::shared_mutex& Model::getStateMutex() {
  return mutexState_;
}

quadruped_model::Command& Model::getCommand() {
  return command_;
}

const quadruped_model::Command& Model::getCommand() const {
  return command_;
}

boost::shared_mutex& Model::getCommandMutex() {
  return mutexCommand_;
}

void Model::initializeForControllerFromFile(double dt, bool isRealRobot, const std::string& filePath, const quadruped_model::Quadrupeds& quadruped) {
  boost::unique_lock<boost::shared_mutex> lock(mutexState_);

  quadrupedModel_.reset(new quadruped_model::QuadrupedModel(dt));
  quadrupedModelDesired_.reset(new quadruped_model::QuadrupedModel(dt));

  /* initialize model from URDF decription */
  if(!quadrupedModel_->initModelFromUrdfFile(filePath)) {
    ROCO_ERROR("[Model::initializeForController] Could not initialize quadruped model from urdf file!");
  }

  /* initialize model from URDF decription */
  if(!quadrupedModelDesired_->initModelFromUrdfFile(filePath)) {
    ROCO_ERROR("[Model::initializeForController] Could not initialize desired quadruped model from urdf file!");
  }

  state_.setQuadrupedModelPtr(this->getQuadrupedModel());
  state_.setDesiredQuadrupedModelPtr(quadrupedModelDesired_.get());
  state_.setTerrainPtr(this->getTerrainModel());

  quadruped_model::quadrupeds::initializeState(state_);

  switch (quadruped) {
    case(quadruped_model::Quadrupeds::StarlETH): {
      quadruped_model::quadrupeds::starleth::initializeCommand(command_);
      quadruped_model::quadrupeds::starleth::initializeLegConfigurations(*quadrupedModel_);
    } break;
    case(quadruped_model::Quadrupeds::Anymal): {
      quadruped_model::quadrupeds::anymal::initializeCommand(command_);
    } break;
    default: {
      throw std::runtime_error("[Model::initializeForController] Invalid quadruped enum.");
    } break;
  }

  quadrupedModel_->setIsRealRobot(isRealRobot);


}

void Model::initializeForController(double dt, bool isRealRobot, const std::string& urdfDescription, const quadruped_model::Quadrupeds& quadruped) {
  boost::unique_lock<boost::shared_mutex> lock(mutexState_);

  quadrupedModel_.reset(new quadruped_model::QuadrupedModel(dt));
  quadrupedModelDesired_.reset(new quadruped_model::QuadrupedModel(dt));

  /* initialize model from URDF decription */
  if(!quadrupedModel_->initModelFromUrdfString(urdfDescription)) {
    ROCO_ERROR("[Model::initializeForController] Could not initialize quadruped model from urdf description!");
  }

  /* initialize the desired model from URDF decription */
  if(!quadrupedModelDesired_->initModelFromUrdfString(urdfDescription)) {
    ROCO_ERROR("[Model::initializeForController] Could not initialize desired quadruped model from urdf description!");
  }

  state_.setQuadrupedModelPtr(this->getQuadrupedModel());
  state_.setDesiredQuadrupedModelPtr(quadrupedModelDesired_.get());
  state_.setTerrainPtr(this->getTerrainModel());

  quadruped_model::quadrupeds::initializeState(state_);

  switch (quadruped) {
    case(quadruped_model::Quadrupeds::StarlETH): {
      quadruped_model::quadrupeds::starleth::initializeCommand(command_);
      quadruped_model::quadrupeds::starleth::initializeLegConfigurations(*quadrupedModel_);
    } break;
    case(quadruped_model::Quadrupeds::Anymal): {
      quadruped_model::quadrupeds::anymal::initializeCommand(command_);
      //quadruped_model::quadrupeds::anymal::initializeLegConfigurations(*quadrupedModel_);
    } break;
    default: {
      throw std::runtime_error("[Model::initializeForController] Invalid quadruped enum.");
    } break;
  }

  quadrupedModel_->setIsRealRobot(isRealRobot);


  terrain_.reset(new robotTerrain::TerrainPlane());
}


void Model::initializeForStateEstimator(double dt, bool isRealRobot, const std::string& pathToUrdfFile, const quadruped_model::Quadrupeds& quadruped) {

}

void Model::reinitialize(double dt) {
  // fixme
//  robotModel_->init();
}

void Model::addVariablesToLog() {
  // fixme
//  robotModel_->addVariablesToLog();

  command_.addVariablesToLog(true);
  quadrupedModel_->addVariablesToLog(true);

  Eigen::Matrix<std::string, 12,1> names;

  names << "LF_HAA_th", "LF_HFE_th", "LF_KFE_th",
       "RF_HAA_th", "RF_HFE_th", "RF_KFE_th",
       "LH_HAA_th", "LH_HFE_th", "LH_KFE_th",
       "RH_HAA_th", "RH_HFE_th", "RH_KFE_th";
  signal_logger::logger->addDoubleEigenMatrixToLog(state_.getJointPositions().toImplementation(), names);

  names << "LF_HAA_thd", "LF_HFE_thd", "LF_KFE_thd",
       "RF_HAA_thd", "RF_HFE_thd", "RF_KFE_thd",
       "LH_HAA_thd", "LH_HFE_thd", "LH_KFE_thd",
       "RH_HAA_thd", "RH_HFE_thd", "RH_KFE_thd";
  signal_logger::logger->addDoubleEigenMatrixToLog(state_.getJointVelocities().toImplementation(), names);

  names << "LF_HAA_load", "LF_HFE_load", "LF_KFE_load",
       "RF_HAA_load", "RF_HFE_load", "RF_KFE_load",
       "LH_HAA_load", "LH_HFE_load", "LH_KFE_load",
       "RH_HAA_load", "RH_HFE_load", "RH_KFE_load";
  signal_logger::logger->addDoubleEigenMatrixToLog(state_.getJointTorques().toImplementation(), names);


  signal_logger::logger->addDoubleKindrEulerAnglesZyxToLog(stateOrientationWorldToBaseEulerAnglesZyx_, "qEulerZyx", "/rm/q/");

//  Eigen::Matrix<std::string, 4,1> contactNames;
//  contactNames << "LF_CONTACT_FLAG", "RF_CONTACT_FLAG", "LH_CONTACT_FLAG", "RH_CONTACT_FLAG";
//  signal_logger::logger->addIntEigenMatrixToLog(robotModel_->contacts().getCA(), contactNames);

  signal_logger::logger->updateLogger(true);


}

void Model::setQuadrupedState(const quadruped_msgs::QuadrupedState::ConstPtr& quadrupedState) {
  boost::unique_lock<boost::shared_mutex> lock(mutexState_);

  /* set contacts */
  static quadruped_model::Force force;
  static quadruped_model::Vector normal;
  bool isClosed;
  for (int iFoot = 0; iFoot < 4; iFoot++) {
    normal.x() = quadrupedState->contacts[iFoot].normal.x;
    normal.y() = quadrupedState->contacts[iFoot].normal.y;
    normal.z() = quadrupedState->contacts[iFoot].normal.z;

    force.x() = quadrupedState->contacts[iFoot].wrench.force.x;
    force.y() = quadrupedState->contacts[iFoot].wrench.force.y;
    force.z() = quadrupedState->contacts[iFoot].wrench.force.z;

    quadrupedModel_->getContactContainer()[iFoot].setForce(force, quadruped_model::CoordinateFrame::WORLD);
    quadrupedModel_->getContactContainer()[iFoot].setNormal(normal, quadruped_model::CoordinateFrame::WORLD);
    if (quadrupedState->contacts[iFoot].state == quadrupedState->contacts[iFoot].STATE_CLOSED) {
        quadrupedModel_->getContactContainer()[iFoot].setState(quadruped_model::ContactState::CLOSED);
    }
    else if (quadrupedState->contacts[iFoot].state == quadrupedState->contacts[iFoot].STATE_SLIPPING) {
        quadrupedModel_->getContactContainer()[iFoot].setState(quadruped_model::ContactState::SLIPPING);
    }
    else if (quadrupedState->contacts[iFoot].state == quadrupedState->contacts[iFoot].STATE_OPEN) {
        quadrupedModel_->getContactContainer()[iFoot].setState(quadruped_model::ContactState::OPEN);
    }
    else {
        throw std::range_error("Contact state is not supported!");
    }

  }


  static quadruped_model::JointTorques jointTorques;
  static quadruped_model::JointPositions jointPositions;
  static quadruped_model::JointVelocities jointVelocities;

  for (int i = 0; i < jointPositions.toImplementation().size(); i++) {
    jointTorques(i) = quadrupedState->joints.effort[i];
    jointPositions(i) = quadrupedState->joints.position[i];
    jointVelocities(i) = quadrupedState->joints.velocity[i];
  }

  RotationQuaternion orientationWorldToBase(quadrupedState->pose.pose.orientation.w,
                                            quadrupedState->pose.pose.orientation.x,
                                            quadrupedState->pose.pose.orientation.y,
                                            quadrupedState->pose.pose.orientation.z);

  // for logging only
  stateOrientationWorldToBaseEulerAnglesZyx_ = quadruped_model::EulerAnglesZyx(orientationWorldToBase).getUnique();

  quadruped_model::LinearVelocity B_v_B(quadrupedState->twist.twist.linear.x,
                                        quadrupedState->twist.twist.linear.y,
                                        quadrupedState->twist.twist.linear.z);
  const quadruped_model::LinearVelocity I_v_B = orientationWorldToBase.inverseRotate(B_v_B);


  quadruped_model::LocalAngularVelocity localAngularVelocityKindr(quadrupedState->twist.twist.angular.x,
                                                                  quadrupedState->twist.twist.angular.y,
                                                                  quadrupedState->twist.twist.angular.z);


  quadruped_model::QuadrupedState quadrupedModelState;

  //-- Generalized positions
  quadrupedModelState.setPositionWorldToBaseInWorldFrame(quadruped_model::Position(quadrupedState->pose.pose.position.x,
                                                                                   quadrupedState->pose.pose.position.y,
																				   quadrupedState->pose.pose.position.z));
  quadrupedModelState.setOrientationWorldToBase(orientationWorldToBase);
  quadrupedModelState.setJointPositions(jointPositions);
  //--

  //-- Generalized velocities
  quadrupedModelState.setLinearVelocityBaseInWorldFrame(I_v_B);
  quadrupedModelState.setAngularVelocityBaseInBaseFrame(localAngularVelocityKindr);
  quadrupedModelState.setJointVelocities(jointVelocities);
  //--

  quadrupedModel_->setJointTorques(jointTorques);
  quadrupedModel_->setState(quadrupedModelState, true, true, false);

  state_.copyStateFromQuadrupedModel();
  state_.setStatus((quadruped_model::State::StateStatus)quadrupedState->state);
}

// used by state_estimator_rm
void Model::setQuadrupedState(const sensor_msgs::ImuPtr& imu,
                   const sensor_msgs::JointStatePtr& jointState,
                   const geometry_msgs::WrenchStampedPtr& contactForceLf,
                   const geometry_msgs::WrenchStampedPtr& contactForceRf,
                   const geometry_msgs::WrenchStampedPtr& contactForceLh,
                   const geometry_msgs::WrenchStampedPtr& contactForceRh,
                   const Eigen::Vector4i& contactFlags) {
  boost::unique_lock<boost::shared_mutex> lock(mutexState_);


  namespace rot = kindr::rotations::eigen_impl;

  static quadruped_model::VectorQb Qb = quadruped_model::VectorQb::Zero();
  static quadruped_model::VectorQb dQb = quadruped_model::VectorQb::Zero();
  static quadruped_model::VectorQb ddQb = quadruped_model::VectorQb::Zero();
  static quadruped_model::VectorAct jointTorques;
  static quadruped_model::VectorQj jointPositions;
  static quadruped_model::VectorQj jointVelocities;

  quadruped_model::Force force;
  quadruped_model::Vector normal = quadruped_model::Vector::UnitZ();


  force.x() = contactForceLf->wrench.force.x;
  force.y() = contactForceLf->wrench.force.y;
  force.z() = contactForceLf->wrench.force.z;
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LF_FOOT].setForce(force);
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LF_FOOT].setNormal(normal);
  if (contactFlags(0) == 1.0) {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LF_FOOT].setState(quadruped_model::ContactState::CLOSED);
  }
  else {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LF_FOOT].setState(quadruped_model::ContactState::OPEN);
  }

  force.x() = contactForceRf->wrench.force.x;
  force.y() = contactForceRf->wrench.force.y;
  force.z() = contactForceRf->wrench.force.z;
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RF_FOOT].setForce(force);
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RF_FOOT].setNormal(normal);
  if (contactFlags(0) == 1.0) {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RF_FOOT].setState(quadruped_model::ContactState::CLOSED);
  }
  else {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RF_FOOT].setState(quadruped_model::ContactState::OPEN);
  }

  force.x() = contactForceLh->wrench.force.x;
  force.y() = contactForceLh->wrench.force.y;
  force.z() = contactForceLh->wrench.force.z;
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LH_FOOT].setForce(force);
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LH_FOOT].setNormal(normal);
  if (contactFlags(0) == 1.0) {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LH_FOOT].setState(quadruped_model::ContactState::CLOSED);
  }
  else {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::LH_FOOT].setState(quadruped_model::ContactState::OPEN);
  }

  force.x() = contactForceRh->wrench.force.x;
  force.y() = contactForceRh->wrench.force.y;
  force.z() = contactForceRh->wrench.force.z;
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RH_FOOT].setForce(force);
  quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RH_FOOT].setNormal(normal);
  if (contactFlags(0) == 1.0) {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RH_FOOT].setState(quadruped_model::ContactState::CLOSED);
  }
  else {
    quadrupedModel_->getContactContainer()[quadruped_model::ContactEnum::RH_FOOT].setState(quadruped_model::ContactState::OPEN);
  }

  for (int i = 0; i < jointPositions.size(); i++) {
    jointTorques(i) =  jointState->effort[i];
    jointPositions(i) = jointState->position[i];
    jointVelocities(i) = jointState->velocity[i];
  }

  Eigen::Vector3d accData;
  accData.x() = imu->linear_acceleration.x;
  accData.y() = imu->linear_acceleration.y;
  accData.z() = imu->linear_acceleration.z;
  //quadrupedModel_->getSensors().getIMU()->setAccelerometerData(accData);
  Eigen::Vector3d gyrData;
  gyrData.x() = imu->angular_velocity.x;
  gyrData.y() = imu->angular_velocity.y;
  gyrData.z() = imu->angular_velocity.z;
  //quadrupedModel_->getSensors().getIMU()->setGyrometerData(gyrData);

  /* update robot model */
//  quadrupedModel_->getSensors().setJointTorques(jointTorques);
//  quadrupedModel_->getSensors().setJointPos(jointPositions);
//  quadrupedModel_->getSensors().setJointVel(jointVelocities);

//  quadrupedModel_->update();
  updateStamp_ = ros::Time::now();
  state_.copyStateFromQuadrupedModel();
}

void Model::initializeQuadrupedState(quadruped_msgs::QuadrupedStatePtr& quadrupedState) const {
  quadruped_description::initializeQuadrupedState(*quadrupedState);
}

void Model::initializeJointState(sensor_msgs::JointState& jointState) const {
  quadruped_description::initializeJointState(jointState);
}

// used by state_estimator_rm
void Model::getQuadrupedState(quadruped_msgs::QuadrupedStatePtr& quadrupedState) {
  boost::shared_lock<boost::shared_mutex> lock(mutexState_);
//  namespace rot = kindr::rotations::eigen_impl;
//
//  const RotationQuaternion  orientationWorldToBase(RotationMatrix(quadrupedModel_->getOrientationWorldToBody(quadruped_model::BodyEnum::BASE)));
//  const Position positionWorldToBaseInWorldFrame = Position(quadrupedModel_->getPositionWorldToBody(quadruped_model::BodyEnum::BASE));
//
//  const LinearVelocity linearVelocityBaseInWorldFrame(quadrupedModel_->getMainBodyGeneralizedVelocities().block<3,1>(0,0));
//  const LinearVelocity linearVelocityBaseInBaseFrame = orientationWorldToBase.rotate(linearVelocityBaseInWorldFrame);
//  const LocalAngularVelocity angularVelocityBaseInBaseFrame(quadrupedModel_->getMainBodyLocalAngularVelocity());
//
//  quadrupedState->header.stamp = updateStamp_;
//
//  quadrupedState->pose.position.x = positionWorldToBaseInWorldFrame.x();
//  quadrupedState->pose.position.y = positionWorldToBaseInWorldFrame.y();
//  quadrupedState->pose.position.z = positionWorldToBaseInWorldFrame.z();
//
//  quadrupedState->pose.orientation.w = orientationWorldToBase.w();
//  quadrupedState->pose.orientation.x = orientationWorldToBase.x();
//  quadrupedState->pose.orientation.y = orientationWorldToBase.y();
//  quadrupedState->pose.orientation.z = orientationWorldToBase.z();
//
//  quadrupedState->twist.linear.x = linearVelocityBaseInBaseFrame.x();
//  quadrupedState->twist.linear.y = linearVelocityBaseInBaseFrame.y();
//  quadrupedState->twist.linear.z = linearVelocityBaseInBaseFrame.z();
//
//  quadrupedState->twist.angular.x = angularVelocityBaseInBaseFrame.x();
//  quadrupedState->twist.angular.y = angularVelocityBaseInBaseFrame.y();
//  quadrupedState->twist.angular.z = angularVelocityBaseInBaseFrame.z();
//
//  quadrupedState->joints.header.stamp = updateStamp_;
//
//  JointPositions  jointPositions(quadrupedModel_->getJointPositions());
//  JointVelocities jointVelocities(quadrupedModel_->getJointVelocities());
//  JointTorques    jointTorques(quadrupedModel_->getJointTorques());
//
//  for (int i=0; i<jointPositions.Dimension; i++) {
//    quadrupedState->joints.position[i]  = jointPositions(i);
//    quadrupedState->joints.velocity[i]  = jointVelocities(i);
//    quadrupedState->joints.effort[i]    = jointTorques(i);
//  }
//
//  for (int i=0; i<quadrupedState->contacts.size(); i++) {
//    quadrupedState->contacts[i].header.stamp = updateStamp_;
//
//    const Force force(quadrupedModel_->getSensors().getContactForceCSw(i));
//    quadrupedState->contacts[i].wrench.force.x = force.x();
//    quadrupedState->contacts[i].wrench.force.y = force.y();
//    quadrupedState->contacts[i].wrench.force.z = force.z();
//
//    Vector normal(quadrupedModel_->getSensors().getContactNormalCSw(i));
//    quadrupedState->contacts[i].normal.x = normal.x();
//    quadrupedState->contacts[i].normal.y = normal.y();
//    quadrupedState->contacts[i].normal.z = normal.z();
//
//    Position position(robotModel_->contacts().getCP(robot_model::CP_LF_World2Foot_CSw+i)->getPos());
//    quadrupedState->contacts[i].position.x = position.x();
//    quadrupedState->contacts[i].position.y = position.y();
//    quadrupedState->contacts[i].position.z = position.z();
//
//    if (robotModel_->contacts().getCP(robot_model::CP_LF_World2Foot_CSw+i)->getFlag()) {
////      std::cout << "leg " << std::to_string(i) << "vel: " << getLinearVelocityFootInBaseFrame(i).norm() << std::endl;
//      if (robotModel_->contacts().getCP(robot_model::CP_LF_World2Foot_CSw+i)->getVel().norm() < 0.01) {
//        quadrupedState->contacts[i].state = quadrupedState->contacts[i].STATE_CLOSED;
//      }
//      else {
//        quadrupedState->contacts[i].state = quadrupedState->contacts[i].STATE_SLIPPING;
//      }
//    } else {
//      quadrupedState->contacts[i].state = quadrupedState->contacts[i].STATE_OPEN;
//    }
//  }
//
}



void Model::getSeActuatorCommands(series_elastic_actuator_msgs::SeActuatorCommandsPtr& actuatorCommands) {
  boost::shared_lock<boost::shared_mutex> lock(mutexCommand_);

  ros::Time stamp = ros::Time::now();
  int i = 0;
  for (auto& command : command_.getActuatorCommands()) {
    actuatorCommands->commands[i].header.stamp = stamp;
    series_elastic_actuator_ros::ConvertRosMessages::writeToMessage(actuatorCommands->commands[i], command);
    ++i;
  }
}

void Model::getSeActuatorCommands(series_elastic_actuator_msgs::SeActuatorCommands& actuatorCommands) {
  boost::shared_lock<boost::shared_mutex> lock(mutexCommand_);

  ros::Time stamp = ros::Time::now();
  int i = 0;
  for (auto& command : command_.getActuatorCommands()) {
    actuatorCommands.commands[i].header.stamp = stamp;
    series_elastic_actuator_ros::ConvertRosMessages::writeToMessage(actuatorCommands.commands[i], command);
    ++i;
  }
}

void Model::getPose(geometry_msgs::PoseWithCovarianceStampedPtr& pose) {
  //fixme
//  const Position positionWorldToBaseInWorldFrame = Position(quadrupedModel_->getPositionWorldToBody(quadruped_model::BodyEnum::BASE));
//  const RotationQuaternion  orientationWorldToBase(RotationMatrix(quadrupedModel_->getOrientationWorldToBody(quadruped_model::BodyEnum::BASE)));
//
//  pose->header.stamp = updateStamp_;
//  pose->pose.pose.position.x = positionWorldToBaseInWorldFrame.x();
//  pose->pose.pose.position.y =  positionWorldToBaseInWorldFrame.y();
//  pose->pose.pose.position.z =  positionWorldToBaseInWorldFrame.z();
//  pose->pose.pose.orientation.w = orientationWorldToBase.w();
//  pose->pose.pose.orientation.x = orientationWorldToBase.x();
//  pose->pose.pose.orientation.y = orientationWorldToBase.y();
//  pose->pose.pose.orientation.z = orientationWorldToBase.z();
//
//  robot_model::EstimatorBase::CovarianceMatrix covariance; // = robotModel_->est().getActualEstimator()->getEstimateCovariance();
//
//  // Variances of pose and twist
//  Eigen::Matrix<double, 12, 6, Eigen::RowMajor> varianceSlMessage;
//  varianceSlMessage.topRows(6) = covariance.block<6,6>(0,0);
//  varianceSlMessage.bottomRows(6) = covariance.block<6,6>(6,6);
//
//  unsigned int covarianceSize = 36;
//
//  for (int i = 0; i < covarianceSize; i++) {
//    pose->pose.covariance.elems[i] = varianceSlMessage(i);
//  }


}

void Model::getTwist(geometry_msgs::TwistWithCovarianceStampedPtr& pose) {
  // todo: implement
}


void Model::setJoystickCommands(const sensor_msgs::Joy& joy) {
  boost::unique_lock<boost::shared_mutex> lock(mutexState_);
  for (int i=0; i<joy.axes.size();i++) {
    state_.getJoystickPtr()->setAxis(i+1, joy.axes[i]);
  }
  for (int i=0; i<joy.buttons.size();i++) {
    state_.getJoystickPtr()->setButton(i+1, joy.buttons[i]);
  }
}


void Model::setCommandVelocity(const geometry_msgs::Twist& twist) {
  boost::unique_lock<boost::shared_mutex> lock(mutexState_);
  state_.getDesiredRobotVelocityPtr()->setDesSagittalVelocity(twist.linear.x);
  state_.getDesiredRobotVelocityPtr()->setDesCoronalVelocity(twist.linear.y);
  state_.getDesiredRobotVelocityPtr()->setDesTurningRate(twist.angular.z);
}

void Model::setMocapData(const geometry_msgs::TransformStamped::ConstPtr& msg) {
//  quadrupedModel_->getSensors().getMocap()->setTimeStamp(msg->header.stamp.toSec());
//  quadrupedModel_->getSensors().getMocap()->setTranslation(Eigen::Vector3d(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
//  quadrupedModel_->getSensors().getMocap()->setRotation(Eigen::Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z));
}

void Model::setSeActuatorReadings(const series_elastic_actuator_msgs::SeActuatorReadings::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(mutexState_);
  for (int i=0; i<msg->readings.size(); i++) {
    setSeActuatorState(i, msg->readings[i].state);
    setSeActuatorCommanded(i, msg->readings[i].commanded);
  }
}

void Model::setSeActuatorState(const int iJoint, const series_elastic_actuator_msgs::SeActuatorState& state) {
//  quadrupedModel_->getSensors().getMotorPos()(iJoint) = state.actuatorPosition;
//  quadrupedModel_->getSensors().getMotorVel()(iJoint) = state.actuatorVelocity;
//  quadrupedModel_->getSensors().getMotorCurrents()(iJoint) = state.current;
}

void Model::setSeActuatorCommanded(const int iJoint, const series_elastic_actuator_msgs::SeActuatorCommand& commanded) {
//  quadrupedModel_->getSensors().getDesiredMotorVel()(iJoint) = commanded.actuatorVelocity;
}


} /* namespace model */
