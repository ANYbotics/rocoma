/*
 * LocomotionController.hpp
 *
 *  Created on: Oct 30, 2014
 *      Author: gech
 */

#ifndef LOCOMOTIONCONTROLLER_HPP_
#define LOCOMOTIONCONTROLLER_HPP_

#include <ros/ros.h>
#include <starleth_msgs/RobotState.h>
#include <sensor_msgs/Joy.h>
#include <starleth_msgs/SeActuatorCommands.h>
#include <locomotion_controller/SwitchController.h>

#include "RobotModel.hpp"
#include "TaskRobotBase.hpp"
#include "NoTask_Task.hpp"

#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/rotations/RotationDiffEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

#include <memory>



namespace locomotion_controller {

class LocomotionController
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
  LocomotionController(ros::NodeHandle& nodeHandle);
  virtual ~LocomotionController();

  bool initialize();
  bool run();


 protected:
  void publish();
  void robotStateCallback(const starleth_msgs::RobotState::ConstPtr& msg);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void setRobotModelParameters();
  void updateRobotModelFromRobotState(const starleth_msgs::RobotState::ConstPtr& robotState);
  void runTask();
  void setupTasks();
  bool switchController(locomotion_controller::SwitchController::Request  &req,
                                 locomotion_controller::SwitchController::Response &res);
 private:
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber robotStateSubscriber_;
  ros::Subscriber joystickSubscriber_;
  ros::Publisher jointCommandsPublisher_;
  ros::ServiceServer switchControllerService_;

  starleth_msgs::SeActuatorCommandsPtr jointCommands_;
  double timeStep_;
  bool isInitializingTask_;
  bool isInitializingNoTask_;
  bool isTaskActive_;
  double time_;
  std::shared_ptr<robotModel::RobotModel> robotModel_;
  std::shared_ptr<robotTerrain::TerrainBase> terrain_;
  /* Create no task, which is active until estimator converged*/
  std::shared_ptr<robotTask::NoTask> noTask_;

  /* Create control task */
  std::shared_ptr<robotTask::TaskRobotBase> task_;



};

} /* namespace locomotion_controller */

#endif /* LOCOMOTIONCONTROLLER_HPP_ */
