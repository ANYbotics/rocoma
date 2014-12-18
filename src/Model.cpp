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
 * @author  Christian Gehring
 * @date    Oct, 2014
 */

#include "locomotion_controller/Model.hpp"
#include "robotUtils/terrains/TerrainPlane.hpp"
#include <starleth_robot_description/starleth_robot_state.hpp>
#include <starlethModel/starleth/starleth.hpp>
#include <robotUtils/loggers/logger.hpp>

namespace model {

Model::Model():
  updateStamp_(0.0),
  robotModel_(),
  terrain_(),
  state_(),
  command_()
{


}

Model::~Model()
{

}


robotModel::RobotModel* Model::getRobotModel() {
  return robotModel_.get();
}
robotTerrain::TerrainBase* Model::getTerrainModel() {
  return terrain_.get();
}

robotModel::State& Model::getState() {
  return state_;
}
robotModel::Command& Model::getCommand() {
  return command_;
}

const robotModel::State& Model::getState() const {
  return state_;
}
const robotModel::Command& Model::getCommand() const {
  return command_;
}

void Model::initializeForController(double dt) {
  robotModel_.reset(new robotModel::RobotModel(dt));


  state_.setRobotModelPtr(this->getRobotModel());
  state_.setTerrainPtr(this->getTerrainModel());
  command_.setRobotModelPtr(this->getRobotModel());
  robotModel::initializeStateForStarlETH(state_);
  robotModel::initializeCommandForStarlETH(command_);

//  setRobotModelParameters();



  robotModel_->setIsRealRobot(false);
  /* Select estimator: */
  robotModel_->est().setActualEstimator(robotModel::PE_SL); // feed through of simulated states
  //  robotModel.est().setActualEstimator(robotModel::PE_ObservabilityConstrainedEKF); // activate this estimator
  //  robotModel.est().setActualEstimator(robotModel::PE_LSE); // not used

  /* activate sensor noise */
  robotModel_->sensors().setIsAddingSensorNoise(false);
  robotModel_->act().setUseLimitsInSimFlag(false); // do not change! see jointController for activating limits
  /* initialize robot model */
  robotModel_->init();

  terrain_.reset(new robotTerrain::TerrainPlane());

}


void Model::initializeForStateEstimator(double dt) {
  robotModel_.reset(new robotModel::RobotModel(dt));
  //setRobotModelParameters();
  state_.setRobotModelPtr(this->getRobotModel());
  state_.setTerrainPtr(this->getTerrainModel());
  command_.setRobotModelPtr(this->getRobotModel());
  robotModel::initializeStateForStarlETH(state_);
  robotModel::initializeCommandForStarlETH(command_);


  robotModel_->setIsRealRobot(false);
  /* Select estimator: */
  robotModel_->est().setActualEstimator(robotModel::PE_ObservabilityConstrainedEKF); // feed through of simulated states
  //  robotModel.est().setActualEstimator(robotModel::PE_ObservabilityConstrainedEKF); // activate this estimator
  //  robotModel.est().setActualEstimator(robotModel::PE_LSE); // not used

  /* activate sensor noise */
  robotModel_->sensors().setIsAddingSensorNoise(false);
  robotModel_->act().setUseLimitsInSimFlag(false); // do not change! see jointController for activating limits
  /* initialize robot model */
  robotModel_->init();

  terrain_.reset(new robotTerrain::TerrainPlane());

}

void Model::reinitialize(double dt) {
  robotModel_->init();
}

void Model::addVariablesToLog() {
  robotModel_->addVariablesToLog();


  Eigen::Matrix<std::string, 12,1> names;
  names << "LF_HAA_des_th", "LF_HFE_des_th", "LF_KFE_des_th",
       "RF_HAA_des_th", "RF_HFE_des_th", "RF_KFE_des_th",
       "LH_HAA_des_th", "LH_HFE_des_th", "LH_KFE_des_th",
       "RH_HAA_des_th", "RH_HFE_des_th", "RH_KFE_des_th";
  robotUtils::logger->addDoubleEigenMatrixToLog(command_.getDesiredJointPositions().toImplementation(), names);

  names << "LF_HAA_des_thd", "LF_HFE_des_thd", "LF_KFE_des_thd",
      "RF_HAA_des_thd", "RF_HFE_des_thd", "RF_KFE_des_thd",
      "LH_HAA_des_thd", "LH_HFE_des_thd", "LH_KFE_des_thd",
      "RH_HAA_des_thd", "RH_HFE_des_thd", "RH_KFE_des_thd";
  robotUtils::logger->addDoubleEigenMatrixToLog(command_.getDesiredMotorVelocities().toImplementation(), names);

  names << "LF_HAA_uff", "LF_HFE_uff", "LF_KFE_uff",
       "RF_HAA_uff", "RF_HFE_uff", "RF_KFE_uff",
       "LH_HAA_uff", "LH_HFE_uff", "LH_KFE_uff",
       "RH_HAA_uff", "RH_HFE_uff", "RH_KFE_uff";
  robotUtils::logger->addDoubleEigenMatrixToLog(command_.getDesiredJointTorques().toImplementation(), names);


  names << "LF_HAA_th", "LF_HFE_th", "LF_KFE_th",
       "RF_HAA_th", "RF_HFE_th", "RF_KFE_th",
       "LH_HAA_th", "LH_HFE_th", "LH_KFE_th",
       "RH_HAA_th", "RH_HFE_th", "RH_KFE_th";
//  robotUtils::logger->addToLog(measPositions, names);
  robotUtils::logger->addDoubleEigenMatrixToLog(state_.getJointPositions().toImplementation(), names);

  names << "LF_HAA_thd", "LF_HFE_thd", "LF_KFE_thd",
       "RF_HAA_thd", "RF_HFE_thd", "RF_KFE_thd",
       "LH_HAA_thd", "LH_HFE_thd", "LH_KFE_thd",
       "RH_HAA_thd", "RH_HFE_thd", "RH_KFE_thd";
//  robotUtils::logger->addToLog(measVelocities, names);
  robotUtils::logger->addDoubleEigenMatrixToLog(state_.getJointVelocities().toImplementation(), names);

  names << "LF_HAA_load", "LF_HFE_load", "LF_KFE_load",
       "RF_HAA_load", "RF_HFE_load", "RF_KFE_load",
       "LH_HAA_load", "LH_HFE_load", "LH_KFE_load",
       "RH_HAA_load", "RH_HFE_load", "RH_KFE_load";
//  robotUtils::logger->addToLog(measJointTorques_, names);
  robotUtils::logger->addDoubleEigenMatrixToLog(state_.getJointTorques().toImplementation(), names);


  Eigen::Matrix<std::string, 4,1> contactNames;
  contactNames << "LF_CONTACT_FLAG", "RF_CONTACT_FLAG", "LH_CONTACT_FLAG", "RH_CONTACT_FLAG";
  robotUtils::logger->addIntEigenMatrixToLog(robotModel_->contacts().getCA(), contactNames);

  robotUtils::logger->updateLogger(true);


}

void Model::setRobotState(const starleth_msgs::RobotState::ConstPtr& robotState) {

  namespace rot = kindr::rotations::eigen_impl;

  static robotModel::VectorQb Qb = robotModel::VectorQb::Zero();
  static robotModel::VectorQb dQb = robotModel::VectorQb::Zero();
  static robotModel::VectorQb ddQb = robotModel::VectorQb::Zero();
  static robotModel::VectorAct jointTorques;
  static robotModel::VectorQj jointPositions;
  static robotModel::VectorQj jointVelocities;

  Qb(0) = robotState->pose.position.x;
  Qb(1) = robotState->pose.position.y;
  Qb(2) = robotState->pose.position.z;


  RotationQuaternion orientationWorldToBase(robotState->pose.orientation.w,
                                            robotState->pose.orientation.x,
                                            robotState->pose.orientation.y,
                                            robotState->pose.orientation.z);

  Qb.segment<3>(3) = rot::EulerAnglesXyzPD(orientationWorldToBase).vector();


  dQb(0) = robotState->twist.linear.x;
  dQb(1) = robotState->twist.linear.y;
  dQb(2) = robotState->twist.linear.z;


//  static mule::Vector3d globalAngularVelocity;
//  simulationManager.getRobotBaseAngularVelocityInWorldCoordinates(
//      globalAngularVelocity, idStarleth);


   LocalAngularVelocity localAngularVelocityKindr(robotState->twist.angular.x,
                                             robotState->twist.angular.y,
                                             robotState->twist.angular.z);
    const Eigen::Vector3d drpy = rot::EulerAnglesXyzDiffPD(
        rot::EulerAnglesXyzPD(orientationWorldToBase), localAngularVelocityKindr)
        .toImplementation();

    dQb.tail(3) = drpy;

    Eigen::Vector3d globalAngularVelocity = orientationWorldToBase.inverseRotate(localAngularVelocityKindr.toImplementation());


  /* set contacts */
  static Eigen::Vector4i contactFlags;
  static Eigen::Vector3d force;
  static Eigen::Vector3d normal;
  bool isClosed;
  for (int iFoot = 0; iFoot < 4; iFoot++) {
    contactFlags(iFoot) =  (robotState->contacts[iFoot].state == robotState->contacts[iFoot].STATE_CLOSED ||
                            robotState->contacts[iFoot].state == robotState->contacts[iFoot].STATE_SLIPPING ) ? 1 : 0;

    normal.x() = robotState->contacts[iFoot].normal.x;
    normal.y() = robotState->contacts[iFoot].normal.y;
    normal.z() = robotState->contacts[iFoot].normal.z;

    force.x() = robotState->contacts[iFoot].wrench.force.x;
    force.y() = robotState->contacts[iFoot].wrench.force.y;
    force.z() = robotState->contacts[iFoot].wrench.force.z;

    robotModel_->sensors().setContactForceCSw(iFoot, force);
    robotModel_->sensors().setContactNormalCSw(iFoot, normal);

  }
  for (int i = 0; i < jointPositions.size(); i++) {
    jointTorques(i) = robotState->joints.effort[i];
    jointPositions(i) = robotState->joints.position[i];
    jointVelocities(i) = robotState->joints.velocity[i];
  }

  /* update robot model */
  robotModel_->sensors().setJointTorques(jointTorques);
  robotModel_->sensors().setJointPos(jointPositions);
  robotModel_->sensors().setJointVel(jointVelocities);
//  robotModel_->sensors().setJointAcc(jointAccelerations);
  robotModel_->sensors().getSimMainBodyPose()->setQb(Qb);
  robotModel_->sensors().getSimMainBodyPose()->setdQb(dQb);
  // todo: acceleration is missing!
//  robotModel_.sensors().getSimMainBodyPose()->setddQb(ddQb);

  const Eigen::Vector3d I_v_B = dQb.block<3,1>(0,0);
  const Eigen::Vector3d B_v_B = orientationWorldToBase.rotate(I_v_B);
  robotModel_->sensors().getSimMainBodyPose()->setLinearVelocityBaseInWorldFrame(I_v_B);
  robotModel_->sensors().getSimMainBodyPose()->setLinearVelocityBaseInBaseFrame(B_v_B);
  robotModel_->sensors().getSimMainBodyPose()->setAngularVelocityBaseInWorldFrame(globalAngularVelocity);
  robotModel_->sensors().getSimMainBodyPose()->setAngularVelocityBaseInBaseFrame(localAngularVelocityKindr.toImplementation());
  // todo: acceleration is missing!
//  robotModel_->sensors().getSimMainBodyPose()->setLinearAccelerationBaseInWorldFrame(I_a_B);

  robotModel_->sensors().updateSimulatedIMU();
  robotModel_->sensors().setContactFlags(contactFlags);

  robotModel_->update();
  state_.copyStateFromRobotModel();
}

void Model::setRobotState(const sensor_msgs::ImuPtr& imu,
                   const sensor_msgs::JointStatePtr& jointState,
                   const geometry_msgs::WrenchStampedPtr& contactForceLf,
                   const geometry_msgs::WrenchStampedPtr& contactForceRf,
                   const geometry_msgs::WrenchStampedPtr& contactForceLh,
                   const geometry_msgs::WrenchStampedPtr& contactForceRh) {



  namespace rot = kindr::rotations::eigen_impl;

  static robotModel::VectorQb Qb = robotModel::VectorQb::Zero();
  static robotModel::VectorQb dQb = robotModel::VectorQb::Zero();
  static robotModel::VectorQb ddQb = robotModel::VectorQb::Zero();
  static robotModel::VectorAct jointTorques;
  static robotModel::VectorQj jointPositions;
  static robotModel::VectorQj jointVelocities;

  Eigen::Vector3d force;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector4i contactFlags = Eigen::Vector4i::Zero();

  const double contactForceThreshold = 2.0;

  force.x() = contactForceLf->wrench.force.x;
  force.y() = contactForceLf->wrench.force.y;
  force.z() = contactForceLf->wrench.force.z;
  robotModel_->sensors().setContactForceCSw(0, force);
  robotModel_->sensors().setContactNormalCSw(0, normal);
  contactFlags(0) = force.norm() >= contactForceThreshold ? 1 : 0;

  force.x() = contactForceRf->wrench.force.x;
  force.y() = contactForceRf->wrench.force.y;
  force.z() = contactForceRf->wrench.force.z;
  robotModel_->sensors().setContactForceCSw(1, force);
  robotModel_->sensors().setContactNormalCSw(1, normal);
  contactFlags(1) = force.norm() >= contactForceThreshold ? 1 : 0;

  force.x() = contactForceLh->wrench.force.x;
  force.y() = contactForceLh->wrench.force.y;
  force.z() = contactForceLh->wrench.force.z;
  robotModel_->sensors().setContactForceCSw(2, force);
  robotModel_->sensors().setContactNormalCSw(2, normal);
  contactFlags(2) = force.norm() >= contactForceThreshold ? 1 : 0;

  force.x() = contactForceRh->wrench.force.x;
  force.y() = contactForceRh->wrench.force.y;
  force.z() = contactForceRh->wrench.force.z;
  robotModel_->sensors().setContactForceCSw(3, force);
  robotModel_->sensors().setContactNormalCSw(3, normal);
  contactFlags(3) = force.norm() >= contactForceThreshold ? 1 : 0;

  robotModel_->sensors().setContactFlags(contactFlags);

  for (int i = 0; i < jointPositions.size(); i++) {
    jointTorques(i) =  jointState->effort[i];
    jointPositions(i) = jointState->position[i];
    jointVelocities(i) = jointState->velocity[i];
  }


  Eigen::Vector3d accData;
  accData.x() = imu->linear_acceleration.x;
  accData.y() = imu->linear_acceleration.y;
  accData.z() = imu->linear_acceleration.z;
  robotModel_->sensors().getIMU()->setAccelerometerData(accData);
  Eigen::Vector3d gyrData;
  gyrData.x() = imu->angular_velocity.x;
  gyrData.y() = imu->angular_velocity.y;
  gyrData.z() = imu->angular_velocity.z;
  robotModel_->sensors().getIMU()->setGyrometerData(gyrData);

  /* update robot model */
  robotModel_->sensors().setJointTorques(jointTorques);
  robotModel_->sensors().setJointPos(jointPositions);
  robotModel_->sensors().setJointVel(jointVelocities);

  robotModel_->update();
  state_.copyStateFromRobotModel();
}

void Model::initializeRobotState(starleth_msgs::RobotStatePtr& robotState) const {
//  robotState->contacts.clear();
//  robotState->contacts.push_back(starleth_msgs::Contact());
//  robotState->contacts.push_back(starleth_msgs::Contact());
//  robotState->contacts.push_back(starleth_msgs::Contact());
//  robotState->contacts.push_back(starleth_msgs::Contact());
//  robotState->contacts[0].name = "LF";
//  robotState->contacts[1].name = "RF";
//  robotState->contacts[2].name = "LH";
//  robotState->contacts[3].name = "RH";
//  robotState->contacts[0].header.frame_id = "World";
//  robotState->contacts[1].header.frame_id = "World";
//  robotState->contacts[2].header.frame_id = "World";
//  robotState->contacts[3].header.frame_id = "World";
//
//  for (int i=0; i<robotState->contacts.size() ; i++) {
//    robotState->contacts[i].frictionCoefficient = 0.8;
//    robotState->contacts[i].restitutionCoefficient = 0.0;
//  }
//
//  initializeJointState(robotState->joints);
  starleth_robot_description::initializeRobotStateForStarlETH(*robotState);


}

void Model::initializeJointState(sensor_msgs::JointState& jointState) const {
  starleth_robot_description::initializeJointStateForStarlETH(jointState);
//  jointState.name.clear();
//  jointState.position.clear();
//  jointState.velocity.clear();
//  jointState.effort.clear();
//
//  jointState.name.push_back("LF_HAA");
//  jointState.name.push_back("LF_HFE");
//  jointState.name.push_back("LF_KFE");
//  jointState.name.push_back("RF_HAA");
//  jointState.name.push_back("RF_HFE");
//  jointState.name.push_back("RF_KFE");
//  jointState.name.push_back("LH_HAA");
//  jointState.name.push_back("LH_HFE");
//  jointState.name.push_back("LH_KFE");
//  jointState.name.push_back("RH_HAA");
//  jointState.name.push_back("RH_HFE");
//  jointState.name.push_back("RH_KFE");
//
//  for (int i=0; i<12;i++) {
//    jointState.position.push_back(0.0);
//    jointState.velocity.push_back(0.0);
//    jointState.effort.push_back(0.0);
//  }
}

void Model::getRobotState(starleth_msgs::RobotStatePtr& robotState) {
  namespace rot = kindr::rotations::eigen_impl;

  kindr::rotations::eigen_impl::RotationQuaternionAD rquatWorldToBaseActive(
      robotModel_->est().getActualEstimator()->getQuat());
  const RotationQuaternion  orientationWorldToBase = rquatWorldToBaseActive.getPassive();

  const Position positionWorldToBaseInWorldFrame = Position(
      robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos());


  const LinearVelocity linearVelocityBaseInWorldFrame(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getVel());
  const LocalAngularVelocity angularVelocityBaseInBaseFrame(orientationWorldToBase.rotate(robotModel_->est().getOmega()));


  robotState->header.stamp = updateStamp_;

  robotState->pose.position.x = positionWorldToBaseInWorldFrame.x();
  robotState->pose.position.y = positionWorldToBaseInWorldFrame.y();
  robotState->pose.position.z = positionWorldToBaseInWorldFrame.z();


  robotState->pose.orientation.w = orientationWorldToBase.w();
  robotState->pose.orientation.x = orientationWorldToBase.x();
  robotState->pose.orientation.y = orientationWorldToBase.y();
  robotState->pose.orientation.z = orientationWorldToBase.z();

  robotState->twist.linear.x = linearVelocityBaseInWorldFrame.x();
  robotState->twist.linear.y = linearVelocityBaseInWorldFrame.y();
  robotState->twist.linear.z = linearVelocityBaseInWorldFrame.z();

  robotState->twist.angular.x = angularVelocityBaseInBaseFrame.x();
  robotState->twist.angular.y = angularVelocityBaseInBaseFrame.y();
  robotState->twist.angular.z = angularVelocityBaseInBaseFrame.z();

  robotState->joints.header.stamp = updateStamp_;

  JointPositions jointPositions(robotModel_->q().getQj());
  JointVelocities jointVelocities(robotModel_->q().getdQj());
  JointTorques jointTorques(robotModel_->sensors().getJointTorques());
  for (int i=0; i<jointPositions.Dimension; i++) {
    robotState->joints.position[i]= jointPositions(i);
    robotState->joints.velocity[i] = jointVelocities(i);
    robotState->joints.effort[i]= jointTorques(i);
  }


  for (int i=0; i<robotState->contacts.size(); i++) {


    robotState->contacts[i].header.stamp = updateStamp_;

    const Force force(robotModel_->sensors().getContactForceCSw(i));
    robotState->contacts[i].wrench.force.x = force.x();
    robotState->contacts[i].wrench.force.y = force.y();
    robotState->contacts[i].wrench.force.z = force.z();

    Vector normal(robotModel_->sensors().getContactNormalCSw(i));
    robotState->contacts[i].normal.x = normal.x();
    robotState->contacts[i].normal.y = normal.y();
    robotState->contacts[i].normal.z = normal.z();

    Position position(robotModel_->contacts().getCP(robotModel::CP_LF_World2Foot_CSw+i)->getPos());
    robotState->contacts[i].position.x = position.x();
    robotState->contacts[i].position.y = position.y();
    robotState->contacts[i].position.z = position.z();

    if (robotModel_->contacts().getCP(robotModel::CP_LF_World2Foot_CSw+i)->getFlag()) {
//      std::cout << "leg " << std::to_string(i) << "vel: " << getLinearVelocityFootInBaseFrame(i).norm() << std::endl;
      if (robotModel_->contacts().getCP(robotModel::CP_LF_World2Foot_CSw+i)->getVel().norm() < 0.01) {
        robotState->contacts[i].state = robotState->contacts[i].STATE_CLOSED;
      }
      else {
        robotState->contacts[i].state = robotState->contacts[i].STATE_SLIPPING;
      }
    } else {
      robotState->contacts[i].state = robotState->contacts[i].STATE_OPEN;
    }
  }




}



void Model::getSeActuatorCommands(starleth_msgs::SeActuatorCommandsPtr& actuatorCommands) {
  ros::Time stamp = ros::Time::now();
  for (int i=0; i<actuatorCommands->commands.size(); i++) {
    actuatorCommands->commands[i].header.stamp = stamp;
    actuatorCommands->commands[i].mode = command_.getDesiredControlModes()(i);
    actuatorCommands->commands[i].jointPosition = command_.getDesiredJointPositions()(i);
    actuatorCommands->commands[i].motorVelocity = command_.getDesiredMotorVelocities()(i);
    actuatorCommands->commands[i].jointTorque = command_.getDesiredJointTorques()(i);
  }
}

void Model::setJoystickCommands(const sensor_msgs::Joy::ConstPtr& msg) {
  for (int i=0; i<msg->axes.size();i++) {
    robotModel_->sensors().getJoystick()->setAxis(i+1, msg->axes[i]);
  }
  for (int i=0; i<msg->buttons.size();i++) {
    robotModel_->sensors().getJoystick()->setButton(i+1, msg->buttons[i]);
  }
}



} /* namespace model */