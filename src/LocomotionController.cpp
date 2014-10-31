/*
 * LocomotionController.cpp
 *
 *  Created on: Oct 30, 2014
 *      Author: gech
 */

#include "locomotion_controller/LocomotionController.hpp"

#include "robotUtils/terrains/TerrainPlane.hpp"

#ifdef USE_TASK_LOCODEMO
#include "LocoDemo_Task.hpp"
#endif

namespace locomotion_controller {

LocomotionController::LocomotionController(ros::NodeHandle& nodeHandle):
    nodeHandle_(nodeHandle),
    timeStep_(0.0025),
    isInitializingTask_(false),
    time_(0.0),
    robotModel_()
{

}

LocomotionController::~LocomotionController()
{
}

bool LocomotionController::initialize() {
  std::string robotStateTopicName;
  nodeHandle_.param<std::string>("topic/robot_state", robotStateTopicName, "robot_state");
  robotStateSubscriber_ = nodeHandle_.subscribe(robotStateTopicName, 1, &LocomotionController::robotStateCallback, this);

  std::string joystickTopicName;
  nodeHandle_.param<std::string>("topic/joy", joystickTopicName, "joy");
  joystickSubscriber_ = nodeHandle_.subscribe(joystickTopicName, 1, &LocomotionController::joystickCallback, this);

  std::string jointCommandsTopicName;
  nodeHandle_.param<std::string>("topic/joint_commands", jointCommandsTopicName, "joint_commands");
  jointCommandsPublisher_ = nodeHandle_.advertise<starleth_msgs::SeActuatorCommands>(jointCommandsTopicName, 1);

  jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
  for (int i=0; i<jointCommands_->commands.size(); i++) {
    jointCommands_->commands[i].mode =  jointCommands_->commands[i].MODE_MOTOR_VELOCITY;
    jointCommands_->commands[i].motorVelocity = 0.0;
  }

  time_ = 0.0;

  robotModel_.reset(new robotModel::RobotModel(timeStep_));
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


  setupTasks();

  switchControllerService_ = nodeHandle_.advertiseService("switch_controller", &LocomotionController::switchController, this);

  return true;
}

bool LocomotionController::run() {
  ros::spin();
  return true;
}


void LocomotionController::publish()  {
  ros::Time stamp = ros::Time::now();
  for (int i=0; i<jointCommands_->commands.size(); i++) {
    jointCommands_->commands[i].header.stamp = stamp;
    jointCommands_->commands[i].mode = robotModel_->act().getMode()(i);
    jointCommands_->commands[i].jointPosition = robotModel_->act().getPos()(i);
    jointCommands_->commands[i].motorVelocity = robotModel_->act().getVel()(i);
    jointCommands_->commands[i].jointTorque = robotModel_->act().getTau()(i);
  }

  if(jointCommandsPublisher_.getNumSubscribers() > 0u) {
    jointCommandsPublisher_.publish(jointCommands_);
  }
}
void LocomotionController::robotStateCallback(const starleth_msgs::RobotState::ConstPtr& msg) {
//  ROS_INFO("Received state");
  updateRobotModelFromRobotState(msg);
  runTask();

  publish();
}

void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  for (int i=0; i<msg->axes.size();i++) {
    robotModel_->sensors().getJoystick()->setAxis(i+1, msg->axes[i]);
  }
  for (int i=0; i<msg->buttons.size();i++) {
    robotModel_->sensors().getJoystick()->setButton(i+1, msg->buttons[i]);
  }
}

void LocomotionController::setRobotModelParameters() {
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

void LocomotionController::updateRobotModelFromRobotState(const starleth_msgs::RobotState::ConstPtr& robotState) {

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
    contactFlags(iFoot) =  (robotState->contacts[iFoot].state == robotState->contacts[iFoot].STATE_CLOSED) ? 1: 0;

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

void LocomotionController::setupTasks()  {

  terrain_.reset(new robotTerrain::TerrainPlane());

/* Create no task, which is active until estimator converged*/
  controllers_.push_back(new robotTask::NoTask(robotModel_.get()));
  robotTask::TaskRobotBase* controller = &controllers_.back();
  activeController_ = controller;
  controller->setTimeStep(timeStep_);
  if (!controller->add()) {
    throw std::runtime_error("Could not add 'no task'!");
  }
  isInitializingNoTask_ = true;


#ifdef USE_TASK_LOCODEMO
  controllers_.push_back(new robotTask::LocoDemo(robotModel_.get(), terrain_.get()));
  controller = &controllers_.back();
  controller->setTimeStep(timeStep_);
  ROS_INFO("Added Task %s.", controller->getName().c_str());
  if (!controller->add()) {
    throw std::runtime_error("Could not add the task!");
  }
#endif

}

void LocomotionController::runTask() {

  if (isInitializingTask_) {

    /* initialize the task */
    if (!activeController_->initTask()) {
      throw std::runtime_error("Could not initialize the task!");
    }
    isInitializingTask_ = false;
    ROS_INFO("Initialized controller %s", activeController_->getName().c_str());
  }

  activeController_->setTime(time_);
  activeController_->runTask();
  time_ += timeStep_;
}


bool LocomotionController::switchController(locomotion_controller::SwitchController::Request  &req,
                               locomotion_controller::SwitchController::Response &res)
{

  //--- Check if controller is already active
  if (activeController_->getName() == req.name) {
    res.status = res.STATUS_RUNNING;
    return true;
  }

  for (auto& controller : controllers_) {
    if (req.name == controller.getName()) {
      activeController_ = &controller;
      res.status = res.STATUS_SWITCHED;
      isInitializingTask_ = true;
      ROS_INFO("Switched to controller %s", activeController_->getName().c_str());
      return true;
    }
  }
  res.status = res.STATUS_NOTFOUND;

  return true;
}

} /* namespace locomotion_controller */
