/*
 * Model.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: gech
 */

#include "locomotion_controller/Model.hpp"
#include "robotUtils/terrains/TerrainPlane.hpp"

namespace model {

Model::Model():
  robotModel_(),
  terrain_()
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

void Model::initialize(double dt) {
  robotModel_.reset(new robotModel::RobotModel(dt));
  setRobotModelParameters();
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

void Model::setRobotModelParameters() {
#ifdef I
#undef I
#endif
  robotModel_->params().gravity_ = 9.81;

  /* main body */
  robotModel_->params().mainbody_.m = 17.1;
  robotModel_->params().mainbody_.I(0,0) = 0.226;
  robotModel_->params().mainbody_.I(1,1) = 0.393;
  robotModel_->params().mainbody_.I(2,2) = 0.553;
  robotModel_->params().mainbody_.s = 0.0;
  robotModel_->params().mainbody_.b2hx = 0.505/2.0;
  robotModel_->params().mainbody_.b2hy = 0.37/2.0;
  robotModel_->params().mainbody_.l = 2.0 * robotModel_->params().mainbody_.b2hx ;
  robotModel_->params().mainbody_.b = 2.0 * robotModel_->params().mainbody_.b2hy;


  /* hip */
  robotModel_->params().hip_.m = 1.8;
  robotModel_->params().hip_.I(0,0) = 0.009275;
  robotModel_->params().hip_.I(1,1) = 0.003035;
  robotModel_->params().hip_.I(2,2) = 0.007210;
  robotModel_->params().hip_.s =  -0.1480;
  robotModel_->params().hip_.l =  -0.2000;

  /* thigh */
  robotModel_->params().thigh_.m =  0.3100;
  robotModel_->params().thigh_.I(0,0) = 0.00148976;
  robotModel_->params().thigh_.I(1,1) = 0.00142976;
  robotModel_->params().thigh_.I(2,2) =  0.00021500;
  robotModel_->params().thigh_.s = -0.1480;
  robotModel_->params().thigh_.l = -0.2000;
//   printf("thigh length: %lf", robotModel_->params().thigh_.l);

  /* shank */
  robotModel_->params().shank_.m =  0.3200;
  robotModel_->params().shank_.I(0,0) =  0.00112108;
  robotModel_->params().shank_.I(1,1) =  0.00124208;
  robotModel_->params().shank_.I(2,2) =  0.00020000;
  robotModel_->params().shank_.s =  -0.0840;
  robotModel_->params().shank_.l = -0.2350;
//   printf("shank length: %lf", robotModel_->params().shank_.l);
  robotModel_->params().computeTotalMass();

}

void Model::setRobotState(const starleth_msgs::RobotState::ConstPtr& robotState) {

  namespace rot = kindr::rotations::eigen_impl;

  static robotModel::VectorQb Qb = robotModel::VectorQb::Zero();
  static robotModel::VectorQb dQb = robotModel::VectorQb::Zero();
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


   LocalAngularVelocity angularVelocityKindr(robotState->twist.angular.x,
                                             robotState->twist.angular.y,
                                             robotState->twist.angular.z);
    const Eigen::Vector3d drpy = rot::EulerAnglesXyzDiffPD(
        rot::EulerAnglesXyzPD(orientationWorldToBase), angularVelocityKindr)
        .toImplementation();
    dQb.tail(3) = drpy;

    Eigen::Vector3d globalAngularVelocity = orientationWorldToBase.inverseRotate(angularVelocityKindr.toImplementation());


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
//  robotModel.sensors().getSimMainBodyPose()->setddQb(ddQb);
  robotModel_->sensors().getSimMainBodyPose()->setOmega(globalAngularVelocity);

  robotModel_->sensors().updateSimulatedIMU();
  robotModel_->sensors().setContactFlags(contactFlags);

}

void Model::getSeActuatorCommands(starleth_msgs::SeActuatorCommandsPtr& actuatorCommands) {
  ros::Time stamp = ros::Time::now();
  for (int i=0; i<actuatorCommands->commands.size(); i++) {
    actuatorCommands->commands[i].header.stamp = stamp;
    actuatorCommands->commands[i].mode = robotModel_->act().getMode()(i);
    actuatorCommands->commands[i].jointPosition = robotModel_->act().getPos()(i);
    actuatorCommands->commands[i].motorVelocity = robotModel_->act().getVel()(i);
    actuatorCommands->commands[i].jointTorque = robotModel_->act().getTau()(i);
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
