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

  std::string robotStateTopicName;
  getNodeHandle().param<std::string>("topic/robot_state", robotStateTopicName, "robot_state");
  robotStateSubscriber_ = getNodeHandle().subscribe(robotStateTopicName, 100, &LocomotionController::robotStateCallback, this);

  std::string joystickTopicName;
  getNodeHandle().param<std::string>("topic/joy", joystickTopicName, "joy");
  joystickSubscriber_ = getNodeHandle().subscribe(joystickTopicName, 100, &LocomotionController::joystickCallback, this);

  std::string jointCommandsTopicName;
  getNodeHandle().param<std::string>("topic/joint_commands", jointCommandsTopicName, "joint_commands");
  jointCommandsPublisher_ = getNodeHandle().advertise<starleth_msgs::SeActuatorCommands>(jointCommandsTopicName, 100);

  jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
  for (int i=0; i<jointCommands_->commands.size(); i++) {
    jointCommands_->commands[i].mode =  jointCommands_->commands[i].MODE_MOTOR_VELOCITY;
    jointCommands_->commands[i].motorVelocity = 0.0;
  }

  time_ = 0.0;

  model_.initialize(timeStep_);



  setupTasks();

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
//  jointCommands_.reset(new starleth_msgs::SeActuatorCommands);
//  for (int i=0; i<jointCommands_->commands.size(); i++) {
//    jointCommands_->commands[i].mode =  jointCommands_->commands[i].MODE_MOTOR_VELOCITY;
//    jointCommands_->commands[i].motorVelocity = 0.0;
//  }
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
  runTask();

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
//  for (int i=0; i<msg->axes.size();i++) {
//    robotModel_->sensors().getJoystick()->setAxis(i+1, msg->axes[i]);
//  }
//  for (int i=0; i<msg->buttons.size();i++) {
//    robotModel_->sensors().getJoystick()->setButton(i+1, msg->buttons[i]);
//  }

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
    ROS_INFO("Emergency stop by joystick! (status: %d)",res.status);
  }
}


void LocomotionController::setupTasks()  {



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
  std::string reqTaskName = req.name;
  if (req.name == "EmergencyStop") {
    reqTaskName = "No Task";
    return true;
  }

  //--- Check if controller is already active
  if (reqTaskName == activeController_->getName()) {
    res.status = res.STATUS_RUNNING;
    return true;
  }

  for (auto& controller : controllers_) {
    if (reqTaskName == controller.getName()) {
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
