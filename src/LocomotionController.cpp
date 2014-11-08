/*
 * LocomotionController.cpp
 *
 *  Created on: Oct 30, 2014
 *      Author: gech
 */

#include <pluginlib/class_list_macros.h>

#include "locomotion_controller/LocomotionController.hpp"

#include "robotUtils/terrains/TerrainPlane.hpp"

#ifdef USE_TASK_LOCODEMO
#include "LocoDemo_Task.hpp"
#endif

#include <ros/callback_queue.h>

#include <chrono>
#include <cstdint>

NODEWRAP_EXPORT_CLASS(locomotion_controller, locomotion_controller::LocomotionController)


namespace locomotion_controller {

LocomotionController::LocomotionController():
    timeStep_(0.0025),
    isInitializingTask_(false),
    time_(0.0),
    controllers_(),
    activeController_(nullptr)
{

}

LocomotionController::~LocomotionController()
{
}

void LocomotionController::init() {

  getNodeHandle().param<double>("controller/time_step", timeStep_, 0.0025);

  robotStateSubscriber_ = subscribe("robot_state", "/robot", 100, &LocomotionController::robotStateCallback);
  joystickSubscriber_ = subscribe("joy", "/joy", 100, &LocomotionController::joystickCallback);

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



  setupControllers();

  switchControllerService_ = getNodeHandle().advertiseService("switch_controller", &LocomotionController::switchController, this);

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
  updateController();

  publish();

  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();

  int64_t timeStep = (int64_t)(timeStep_*1e9);
  if (elapsedTimeNSecs > timeStep) {
    ROS_INFO("Warning: computation is not real-time! Elapsed time: %lf ms\n", (double)elapsedTimeNSecs*1e-6);
  }

}

void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {

  model_.setJoystickCommands(msg);

  // START + LF buttons
  if (msg->buttons[4] == 1 && msg->buttons[7] == 1 ) {
    locomotion_controller::SwitchController::Request  req;
    locomotion_controller::SwitchController::Response res;
    req.name = "LocoDemo";
    if(!switchController(req,res)) {
    }
    ROS_INFO("Switched task by joystick (status: %d)",res.status);

  }
  // RB button
  if (msg->buttons[5] == 1 ) {
    locomotion_controller::SwitchController::Request  req;
    locomotion_controller::SwitchController::Response res;
    req.name = "EmergencyStop";
    if(!switchController(req,res)) {
    }
    NODEWRAP_INFO("Emergency stop by joystick! (status: %d)",res.status);
  }
}


void LocomotionController::setupControllers()  {



/* Create no task, which is active until estimator converged*/
  controllers_.push_back(new robotTask::NoTask(model_.getRobotModel()));
  robotTask::TaskRobotBase* controller = &controllers_.back();
  activeController_ = controller;
  controller->setTimeStep(timeStep_);
  if (!controller->add()) {
    throw std::runtime_error("Could not add 'no task'!");
  }



#ifdef USE_TASK_LOCODEMO
  controllers_.push_back(new robotTask::LocoDemo(model_.getRobotModel(), model_.getTerrainModel()));
  controller = &controllers_.back();
  controller->setTimeStep(timeStep_);
  ROS_INFO("Added Task %s.", controller->getName().c_str());
  if (!controller->add()) {
    throw std::runtime_error("Could not add the task!");
  }
#endif

}

void LocomotionController::updateController() {

  if (isInitializingTask_) {

    /* initialize the task */
    if (!activeController_->initTask()) {
      throw std::runtime_error("Could not initialize the task!");
    }
    isInitializingTask_ = false;
    NODEWRAP_INFO("Initialized controller %s", activeController_->getName().c_str());
  }

  activeController_->setTime(time_);
  activeController_->runTask();
  time_ += timeStep_;
}


bool LocomotionController::switchController(locomotion_controller::SwitchController::Request  &req,
                               locomotion_controller::SwitchController::Response &res)
{
  std::string reqTaskName = req.name;
  if (req.name == "EmergencyStop") {
    NODEWRAP_INFO("Emergency Stop!");
    reqTaskName = "No Task";
  }

  //--- Check if controller is already active
  if (reqTaskName == activeController_->getName()) {
    res.status = res.STATUS_RUNNING;
    NODEWRAP_INFO("Controller is already running!");
    return true;
  }

  for (auto& controller : controllers_) {
    if (reqTaskName == controller.getName()) {
      activeController_ = &controller;
      res.status = res.STATUS_SWITCHED;
      isInitializingTask_ = true;
      NODEWRAP_INFO("Switched to controller %s", activeController_->getName().c_str());
      return true;
    }
  }
  res.status = res.STATUS_NOTFOUND;
  NODEWRAP_INFO("Controller %s not found!", reqTaskName.c_str());
  return true;
}

} /* namespace locomotion_controller */
