/*
 * LocomotionController.cpp
 *
 *  Created on: Oct 30, 2014
 *      Author: gech
 */

#include <pluginlib/class_list_macros.h>

#include "locomotion_controller/LocomotionController.hpp"

#include "robotUtils/terrains/TerrainPlane.hpp"


#include <ros/callback_queue.h>

#include <chrono>
#include <cstdint>

NODEWRAP_EXPORT_CLASS(locomotion_controller, locomotion_controller::LocomotionController)


namespace locomotion_controller {

LocomotionController::LocomotionController():
    timeStep_(0.0025),
    time_(0.0)
{

}

LocomotionController::~LocomotionController()
{
}

void LocomotionController::init() {

  getNodeHandle().param<double>("controller/time_step", timeStep_, 0.0025);

  robotStateSubscriber_ = subscribe("robot_state", "/robot", 100, &LocomotionController::robotStateCallback, ros::TransportHints().tcpNoDelay());
  joystickSubscriber_ = subscribe("joy", "/joy", 100, &LocomotionController::joystickCallback, ros::TransportHints().tcpNoDelay());

  ros::AdvertiseOptions opSea;
  opSea.init<starleth_msgs::SeActuatorCommands>("command_seactuators", 100);
  jointCommandsPublisher_ = advertise("command_seactuators",opSea);

  jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
  for (int i=0; i<jointCommands_->commands.size(); i++) {
    jointCommands_->commands[i].mode =  jointCommands_->commands[i].MODE_MOTOR_VELOCITY;
    jointCommands_->commands[i].motorVelocity = 0.0;
  }

  time_ = 0.0;

  model_.initialize(timeStep_);
  model_.getRobotModel()->params().printParams();


  controllerManager_.setupControllers(timeStep_, time_, &model_);

  switchControllerService_ = getNodeHandle().advertiseService("switch_controller", &ControllerManager::switchController, &this->controllerManager_);

}

bool LocomotionController::run() {
//  ros::Rate loop_rate(800);
//  while (ros::ok())
//  {
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.0));
////    ros::spinOnce();
////    loop_rate.sleep();
//  }
  ros::spin();
  return true;
}


void LocomotionController::publish()  {

  model_.getSeActuatorCommands(jointCommands_);
//  ros::Time stamp = ros::Time::now();
//  for (int i=0; i<jointCommands_->commands.size(); i++) {
//    jointCommands_->commands[i].header.stamp = stamp;
//    jointCommands_->commands[i].mode = robotModel_->act().getMode()(i);
//    jointCommands_->commands[i].jointPosition = robotModel_->act().getPos()(i);
//    jointCommands_->commands[i].motorVelocity = robotModel_->act().getVel()(i);
//    jointCommands_->commands[i].jointTorque = robotModel_->act().getTau()(i);
//  }

  if(jointCommandsPublisher_.getNumSubscribers() > 0u) {
    jointCommandsPublisher_.publish(jointCommands_);
    ros::spinOnce();
//    ROS_INFO("Publish");
  }

}
void LocomotionController::robotStateCallback(const starleth_msgs::RobotState::ConstPtr& msg) {
//  ROS_INFO("Received state");
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  start = std::chrono::steady_clock::now();

  model_.setRobotState(msg);
  controllerManager_.updateController();

  publish();

  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();

  int64_t timeStep = (int64_t)(timeStep_*1e9);
  if (elapsedTimeNSecs > timeStep) {
    ROS_INFO("Warning: computation is not real-time! Elapsed time: %lf ms\n", (double)elapsedTimeNSecs*1e-6);
  }

  time_ += timeStep_;
}

void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {

  model_.setJoystickCommands(msg);

  // START + LF buttons
  if (msg->buttons[4] == 1 && msg->buttons[7] == 1 ) {
    locomotion_controller::SwitchController::Request  req;
    locomotion_controller::SwitchController::Response res;
    req.name = "LocoDemo";
    if(!controllerManager_.switchController(req,res)) {
    }
    ROS_INFO("Switched task by joystick (status: %d)",res.status);

  }
  // RB button
  if (msg->buttons[5] == 1 ) {
    locomotion_controller::SwitchController::Request  req;
    locomotion_controller::SwitchController::Response res;
    req.name = "EmergencyStop";
    if(!controllerManager_.switchController(req,res)) {
    }
    NODEWRAP_INFO("Emergency stop by joystick! (status: %d)",res.status);
  }
}






} /* namespace locomotion_controller */
