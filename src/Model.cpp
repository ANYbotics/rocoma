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
quadruped_model::Command& Model::getCommand() {
  return command_;
}

const quadruped_model::State& Model::getState() const {
  return state_;
}
const quadruped_model::Command& Model::getCommand() const {
  return command_;
}

void Model::initializeForController(double dt, bool isRealRobot, const std::string& pathToUrdfFile, const quadruped_model::Quadrupeds& quadruped) {
  quadrupedModel_.reset(new quadruped_model::QuadrupedModel(dt));

  /* initialize model from URDF file */
  if(!quadrupedModel_->initModelFromUrdfFile(pathToUrdfFile)) {
    ROCO_ERROR("Could not load URDF file!");
  }

  state_.setQuadrupedModelPtr(this->getQuadrupedModel());
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


//  Eigen::Matrix<std::string, 4,1> contactNames;
//  contactNames << "LF_CONTACT_FLAG", "RF_CONTACT_FLAG", "LH_CONTACT_FLAG", "RH_CONTACT_FLAG";
//  signal_logger::logger->addIntEigenMatrixToLog(robotModel_->contacts().getCA(), contactNames);

  signal_logger::logger->updateLogger(true);


}

void Model::setRobotState(const quadruped_msgs::RobotState::ConstPtr& robotState) {

  namespace rot = kindr::rotations::eigen_impl;

  static quadruped_model::VectorQb Qb = quadruped_model::VectorQb::Zero();
  static quadruped_model::VectorQb dQb = quadruped_model::VectorQb::Zero();
  static quadruped_model::VectorQb ddQb = quadruped_model::VectorQb::Zero();
  static quadruped_model::VectorAct jointTorques;
  static quadruped_model::VectorQj jointPositions;
  static quadruped_model::VectorQj jointVelocities;

  Qb(0) = robotState->pose.position.x;
  Qb(1) = robotState->pose.position.y;
  Qb(2) = robotState->pose.position.z;

  RotationQuaternion orientationWorldToBase(robotState->pose.orientation.w,
                                            robotState->pose.orientation.x,
                                            robotState->pose.orientation.y,
                                            robotState->pose.orientation.z);

  Qb.segment<3>(3) = rot::EulerAnglesXyzPD(orientationWorldToBase).vector();


  Eigen::Vector3d B_v_B;
  B_v_B(0) = robotState->twist.linear.x;
  B_v_B(1) = robotState->twist.linear.y;
  B_v_B(2) = robotState->twist.linear.z;

  const Eigen::Vector3d I_v_B = orientationWorldToBase.inverseRotate(B_v_B);
  dQb.segment<3>(0) = I_v_B;

  LocalAngularVelocity localAngularVelocityKindr(robotState->twist.angular.x,
                                                 robotState->twist.angular.y,
                                                 robotState->twist.angular.z);
  const Eigen::Vector3d drpy = rot::EulerAnglesXyzDiffPD(
      rot::EulerAnglesXyzPD(orientationWorldToBase), localAngularVelocityKindr)
      .toImplementation();

  dQb.tail(3) = drpy;

  Eigen::Vector3d globalAngularVelocity = orientationWorldToBase.inverseRotate(
      localAngularVelocityKindr.toImplementation());

  /* set contacts */


  static quadruped_model::Force force;
  static quadruped_model::Vector normal;
  bool isClosed;
  for (int iFoot = 0; iFoot < 4; iFoot++) {
    normal.x() = robotState->contacts[iFoot].normal.x;
    normal.y() = robotState->contacts[iFoot].normal.y;
    normal.z() = robotState->contacts[iFoot].normal.z;

    force.x() = robotState->contacts[iFoot].wrench.force.x;
    force.y() = robotState->contacts[iFoot].wrench.force.y;
    force.z() = robotState->contacts[iFoot].wrench.force.z;

    quadrupedModel_->getContactContainer()[iFoot].setForce(force, quadruped_model::CoordinateFrame::WORLD);
    quadrupedModel_->getContactContainer()[iFoot].setNormal(normal, quadruped_model::CoordinateFrame::WORLD);
    if (robotState->contacts[iFoot].state == robotState->contacts[iFoot].STATE_CLOSED) {
        quadrupedModel_->getContactContainer()[iFoot].setState(quadruped_model::ContactState::CLOSED);
    }
    else if (robotState->contacts[iFoot].state == robotState->contacts[iFoot].STATE_SLIPPING) {
        quadrupedModel_->getContactContainer()[iFoot].setState(quadruped_model::ContactState::SLIPPING);
    }
    else if (robotState->contacts[iFoot].state == robotState->contacts[iFoot].STATE_OPEN) {
        quadrupedModel_->getContactContainer()[iFoot].setState(quadruped_model::ContactState::OPEN);
    }
    else {
        throw std::range_error("Contact state is not supported!");
    }

  }

  for (int i = 0; i < jointPositions.size(); i++) {
    jointTorques(i) = robotState->joints.effort[i];
    jointPositions(i) = robotState->joints.position[i];
    jointVelocities(i) = robotState->joints.velocity[i];
  }

  quadrupedModel_->setTorsoGeneralizedPositionsEulerXyz(Qb);
  quadrupedModel_->setTorsoGeneralizedVelocitiesEulerXyz(dQb);
  quadrupedModel_->setTorsoLocalAngularVelocity(localAngularVelocityKindr);
//  quadrupedModel_->setMainBodyGlobalAngularVelocity(globalAngularVelocity);

  quadrupedModel_->setJointPositions(jointPositions);
  quadrupedModel_->setJointVelocities(jointVelocities);
  quadrupedModel_->setJointTorques(jointTorques);

  // todo: acceleration is missing!
  //  robotModel_.sensors().getSimMainBodyPose()->setddQb(ddQb);

  quadrupedModel_->updateKinematics(true, true, false);

//  quadrupedModel_->getSensors().getSimMainBodyPose()->setLinearVelocityBaseInWorldFrame(I_v_B);
//  quadrupedModel_->getSensors().getSimMainBodyPose()->setLinearVelocityBaseInBaseFrame(B_v_B);
//  quadrupedModel_->getSensors().getSimMainBodyPose()->setAngularVelocityBaseInWorldFrame(globalAngularVelocity);
//  quadrupedModel_->getSensors().getSimMainBodyPose()->setAngularVelocityBaseInBaseFrame(localAngularVelocityKindr.toImplementation());

  //   todo: acceleration is missing!
//  quadrupedModel_->getSensors().getSimMainBodyPose()->setLinearAccelerationBaseInWorldFrame(I_a_B);



  state_.copyStateFromQuadrupedModel();
  state_.setStatus((quadruped_model::State::StateStatus)robotState->state);
}

// used by state_estimator_rm
void Model::setRobotState(const sensor_msgs::ImuPtr& imu,
                   const sensor_msgs::JointStatePtr& jointState,
                   const geometry_msgs::WrenchStampedPtr& contactForceLf,
                   const geometry_msgs::WrenchStampedPtr& contactForceRf,
                   const geometry_msgs::WrenchStampedPtr& contactForceLh,
                   const geometry_msgs::WrenchStampedPtr& contactForceRh,
                   const Eigen::Vector4i& contactFlags) {
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

void Model::initializeRobotState(quadruped_msgs::RobotStatePtr& robotState) const {
  quadruped_description::initializeRobotState(*robotState);
}

void Model::initializeJointState(sensor_msgs::JointState& jointState) const {
  quadruped_description::initializeJointState(jointState);
}

// used by state_estimator_rm
void Model::getRobotState(quadruped_msgs::RobotStatePtr& robotState) {
//  namespace rot = kindr::rotations::eigen_impl;
//
//  const RotationQuaternion  orientationWorldToBase(RotationMatrix(quadrupedModel_->getOrientationWorldToBody(quadruped_model::BodyEnum::BASE)));
//  const Position positionWorldToBaseInWorldFrame = Position(quadrupedModel_->getPositionWorldToBody(quadruped_model::BodyEnum::BASE));
//
//  const LinearVelocity linearVelocityBaseInWorldFrame(quadrupedModel_->getMainBodyGeneralizedVelocities().block<3,1>(0,0));
//  const LinearVelocity linearVelocityBaseInBaseFrame = orientationWorldToBase.rotate(linearVelocityBaseInWorldFrame);
//  const LocalAngularVelocity angularVelocityBaseInBaseFrame(quadrupedModel_->getMainBodyLocalAngularVelocity());
//
//  robotState->header.stamp = updateStamp_;
//
//  robotState->pose.position.x = positionWorldToBaseInWorldFrame.x();
//  robotState->pose.position.y = positionWorldToBaseInWorldFrame.y();
//  robotState->pose.position.z = positionWorldToBaseInWorldFrame.z();
//
//  robotState->pose.orientation.w = orientationWorldToBase.w();
//  robotState->pose.orientation.x = orientationWorldToBase.x();
//  robotState->pose.orientation.y = orientationWorldToBase.y();
//  robotState->pose.orientation.z = orientationWorldToBase.z();
//
//  robotState->twist.linear.x = linearVelocityBaseInBaseFrame.x();
//  robotState->twist.linear.y = linearVelocityBaseInBaseFrame.y();
//  robotState->twist.linear.z = linearVelocityBaseInBaseFrame.z();
//
//  robotState->twist.angular.x = angularVelocityBaseInBaseFrame.x();
//  robotState->twist.angular.y = angularVelocityBaseInBaseFrame.y();
//  robotState->twist.angular.z = angularVelocityBaseInBaseFrame.z();
//
//  robotState->joints.header.stamp = updateStamp_;
//
//  JointPositions  jointPositions(quadrupedModel_->getJointPositions());
//  JointVelocities jointVelocities(quadrupedModel_->getJointVelocities());
//  JointTorques    jointTorques(quadrupedModel_->getJointTorques());
//
//  for (int i=0; i<jointPositions.Dimension; i++) {
//    robotState->joints.position[i]  = jointPositions(i);
//    robotState->joints.velocity[i]  = jointVelocities(i);
//    robotState->joints.effort[i]    = jointTorques(i);
//  }
//
//  for (int i=0; i<robotState->contacts.size(); i++) {
//    robotState->contacts[i].header.stamp = updateStamp_;
//
//    const Force force(quadrupedModel_->getSensors().getContactForceCSw(i));
//    robotState->contacts[i].wrench.force.x = force.x();
//    robotState->contacts[i].wrench.force.y = force.y();
//    robotState->contacts[i].wrench.force.z = force.z();
//
//    Vector normal(quadrupedModel_->getSensors().getContactNormalCSw(i));
//    robotState->contacts[i].normal.x = normal.x();
//    robotState->contacts[i].normal.y = normal.y();
//    robotState->contacts[i].normal.z = normal.z();
//
//    Position position(robotModel_->contacts().getCP(robot_model::CP_LF_World2Foot_CSw+i)->getPos());
//    robotState->contacts[i].position.x = position.x();
//    robotState->contacts[i].position.y = position.y();
//    robotState->contacts[i].position.z = position.z();
//
//    if (robotModel_->contacts().getCP(robot_model::CP_LF_World2Foot_CSw+i)->getFlag()) {
////      std::cout << "leg " << std::to_string(i) << "vel: " << getLinearVelocityFootInBaseFrame(i).norm() << std::endl;
//      if (robotModel_->contacts().getCP(robot_model::CP_LF_World2Foot_CSw+i)->getVel().norm() < 0.01) {
//        robotState->contacts[i].state = robotState->contacts[i].STATE_CLOSED;
//      }
//      else {
//        robotState->contacts[i].state = robotState->contacts[i].STATE_SLIPPING;
//      }
//    } else {
//      robotState->contacts[i].state = robotState->contacts[i].STATE_OPEN;
//    }
//  }
//
}



void Model::getSeActuatorCommands(series_elastic_actuator_msgs::SeActuatorCommandsPtr& actuatorCommands) {
  ros::Time stamp = ros::Time::now();
  int i = 0;
  for (auto& command : command_.getActuatorCommands()) {
    actuatorCommands->commands[i].header.stamp = stamp;
    series_elastic_actuator_ros::ConvertRosMessages::writeToMessage(actuatorCommands->commands[i], command);
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


void Model::setJoystickCommands(const sensor_msgs::Joy::ConstPtr& msg) {
  for (int i=0; i<msg->axes.size();i++) {
    state_.getJoystickPtr()->setAxis(i+1, msg->axes[i]);
  }
  for (int i=0; i<msg->buttons.size();i++) {
    state_.getJoystickPtr()->setButton(i+1, msg->buttons[i]);
  }
}


void Model::setCommandVelocity(const geometry_msgs::Twist& twist) {
//	 quadrupedModel_->getSensors().getDesRobotVelocity()->setDesSagittalVelocity(twist.linear.x);
//	 quadrupedModel_->getSensors().getDesRobotVelocity()->setDesCoronalVelocity(twist.linear.y);
//	 quadrupedModel_->getSensors().getDesRobotVelocity()->setDesTurningRate(twist.angular.z);

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
