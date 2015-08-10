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
 * @file    LocomotionController.cpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */

#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include "locomotion_controller/LocomotionController.hpp"

#include "locomotion_controller_msgs/SwitchController.h"

// Messages
#include <locomotion_controller_msgs/ResetStateEstimator.h>


#include <robot_model/starleth/starleth.hpp>
#include <starleth_description/starleth_se_actuator_commands.hpp>

#include <signal_logger/logger.hpp>
#include <signal_logger/LoggerNone.hpp>
#include <signal_logger_std/LoggerStd.hpp>
#include <signal_logger_ros/LoggerRos.hpp>

#include <std_srvs/Empty.h>

#include <chrono>
#include <cstdint>
#include <string>

#include <robotUtils/timers/ChronoTimer.hpp>


NODEWRAP_EXPORT_CLASS(locomotion_controller, locomotion_controller::LocomotionController)


namespace locomotion_controller {

LocomotionController::LocomotionController():
    timeStep_(0.0025),
    isRealRobot_(false),
    samplingFrequency_(1.0),
    model_(),
    controllerManager_(this),
    defaultController_("LocoDemo")
{


}

LocomotionController::~LocomotionController()
{
}

void LocomotionController::init() {
  //--- Read parameters.
  getNodeHandle().param<double>("controller/time_step", timeStep_, 0.0025);
  getNodeHandle().param<bool>("controller/is_real_robot", isRealRobot_, false);
  getNodeHandle().param<std::string>("controller/default", defaultController_, "LocoDemo");
  //---

  //--- Configure logger.
  std::string loggingScriptFilename;
  getNodeHandle().param<std::string>("logger/script", loggingScriptFilename, "");
  if (loggingScriptFilename.empty()){
    loggingScriptFilename = ros::package::getPath("locomotion_controller") + std::string{"/config/logging.script"};
  }
  getNodeHandle().param<double>("logger/sampling_time", samplingFrequency_, 60.0);
  NODEWRAP_INFO("Initialize logger with sampling time: %lfs and script: %s.", samplingFrequency_, loggingScriptFilename.c_str());

  std::string loggerClass;
  getNodeHandle().param<std::string>("logger/class", loggerClass, "std");
  if (loggerClass.compare("ros") == 0) {
    // initialize ros logger
    signal_logger::logger.reset(new signal_logger_ros::LoggerRos(getNodeHandle()));
    signal_logger_ros::LoggerRos* loggerRos = static_cast<signal_logger_ros::LoggerRos*>(signal_logger::logger.get());
    loggerRos->setPublishFrequency(samplingFrequency_);
  } else if (loggerClass.compare("std") == 0) {
    // initialize std logger as fallback logger
    signal_logger::logger.reset(new signal_logger_std::LoggerStd());
    signal_logger_std::LoggerStd* loggerStd = static_cast<signal_logger_std::LoggerStd*>(signal_logger::logger.get());
    loggerStd->setVerboseLevel(signal_logger_std::LoggerStd::VL_DEBUG);
  } else {
    signal_logger::logger.reset(new signal_logger::LoggerNone());
  }

  signal_logger::logger->initLogger((int)(1.0/timeStep_), (int)(1.0/timeStep_), samplingFrequency_, loggingScriptFilename);
  NODEWRAP_INFO("Initialize logger with sampling time: %lfs, sampling frequency: %d and script: %s.", samplingFrequency_, (int)(1.0/timeStep_), loggingScriptFilename.c_str());
  //---

  //--- Configure controllers
  {
    std::lock_guard<std::mutex> lockControllerManager(mutexModelAndControllerManager_);

    // Initialize robot and terrain models
    model_.initializeForController(timeStep_,isRealRobot_);
    model_.getRobotModel()->params().printParams();
    model_.addVariablesToLog();

    controllerManager_.setIsRealRobot(isRealRobot_);
    controllerManager_.setupControllers(timeStep_, model_.getState(), model_.getCommand(), getNodeHandle(), this);
  }
  //---

  initializeMessages();
  initializeServices();
  initializePublishers();
  initializeSubscribers();

  /*
   * Start workers
   */
//  controllerWorker_ = addWorker("controller", 400, &LocomotionController::updateControllerWorker);


  nodewrap::WorkerOptions workerOptions;
  workerOptions.callback = boost::bind(&LocomotionController::updateControllerWorker, this, _1);
  workerOptions.frequency = 400;
  workerOptions.autostart = true;
  workerOptions.synchronous = false;
  workerOptions.privateCallbackQueue = true;
  workerOptions.priority = 99;
  controllerWorker_ = addWorker("controller", workerOptions);

}


void LocomotionController::cleanup() {

}


nodewrap::Worker LocomotionController::addWrappedWorker(
    const std::string& name, const nodewrap::WorkerOptions& defaultOptions)
{
  return this->addWorker(name, defaultOptions);
}


double LocomotionController::getSamplingFrequency() const {
  return samplingFrequency_;
}


void LocomotionController::initializeMessages() {
  //--- Initialize joint commands.
  {
    std::lock_guard<std::mutex> lock(mutexJointCommands_);
    jointCommands_.reset(new series_elastic_actuator_msgs::SeActuatorCommands);
    starleth_description::initializeSeActuatorCommandsForStarlETH(*jointCommands_);
  }
  //---
}

void LocomotionController::initializeServices() {
  switchControllerService_ = getNodeHandle().advertiseService("switch_controller", &ControllerManager::switchController, &this->controllerManager_);
  getAvailableControllersService_ = getNodeHandle().advertiseService("get_available_controllers", &ControllerManager::getAvailableControllers, &this->controllerManager_);
  getActiveControllerService_ = getNodeHandle().advertiseService("get_active_controller", &ControllerManager::getActiveController, &this->controllerManager_);
  emergencyStopService_ = advertiseService("emergency_stop", "/emergency_stop", &LocomotionController::emergencyStop);
  resetStateEstimatorClient_ = serviceClient<locomotion_controller_msgs::ResetStateEstimator>("reset_state_estimator", "/reset_state_estimator");
  //resetStateEstimatorClient_.waitForExistence();
}


void LocomotionController::initializePublishers() {

  /*****************************
   * Initialize joint commands *
   *****************************/
  jointCommandsNumSubscribers_ = 0;
  ros::AdvertiseOptions jointCommandsOptions;
  jointCommandsOptions.init<series_elastic_actuator_msgs::SeActuatorCommands>("/command_seactuators",
                                                                              100,
                                                                              boost::bind(&LocomotionController::jointCommandsSubscriberConnect,this,_1),
                                                                              boost::bind(&LocomotionController::jointCommandsSubscriberDisconnect,this,_1));

  //todo: fix bug that prevents the connect and disconnect callbacks from being called
//  jointCommandsCallbackQueue_.reset(new ros::CallbackQueue());
//  jointCommandsOptions.callback_queue = jointCommandsCallbackQueue_.get();

  jointCommandsPublisher_ = advertise("command_seactuators",jointCommandsOptions);
  ROS_INFO_STREAM("commands topic name: " << jointCommandsPublisher_.getTopic());
  /*****************************/

}


void LocomotionController::initializeSubscribers() {
  joystickSubscriber_ = subscribe("joy", "/joy", 100, &LocomotionController::joystickCallback, ros::TransportHints().tcpNoDelay());
  commandVelocitySubscriber_ = subscribe("command_velocity", "/command_velocity", 100, &LocomotionController::commandVelocityCallback, ros::TransportHints().tcpNoDelay());
  //--- temporary
  mocapSubscriber_ = subscribe("mocap", "mocap", 100, &LocomotionController::mocapCallback, ros::TransportHints().tcpNoDelay());
  seActuatorReadingsSubscriber_ = subscribe("actuator_readings", "/actuator_readings", 100, &LocomotionController::seActuatorReadingsCallback, ros::TransportHints().tcpNoDelay());
  //---

  // this should be last since it will start the controller loop
  robotStateSubscriber_ = subscribe("robot_state", "/robot", 100, &LocomotionController::robotStateCallback, ros::TransportHints().tcpNoDelay());

}


void LocomotionController::jointCommandsSubscriberConnect(const ros::SingleSubscriberPublisher& pub) {
  jointCommandsNumSubscribers_++;
  ROS_INFO_STREAM("increasing joint commands to: " << jointCommandsNumSubscribers_);
}
void LocomotionController::jointCommandsSubscriberDisconnect(const ros::SingleSubscriberPublisher& pub) {
  jointCommandsNumSubscribers_--;
  ROS_INFO_STREAM("decreasing joint commands to: " << jointCommandsNumSubscribers_);
}


void LocomotionController::publish()  {

  int64_t timeStep = (int64_t)(timeStep_*1e9);
  robotUtils::ChronoTimer timer;
  timer.pinTime();

  if (jointCommandsNumSubscribers_ > 0u) {

    int64_t nanoSecs = timer.getElapsedTimeNanoSec();
    if (nanoSecs > timeStep) {
      NODEWRAP_WARN("getNumSubscribers: %lf ms\n", (double)nanoSecs*1e-6);
    }


    series_elastic_actuator_msgs::SeActuatorCommandsPtr jointCommands;

    {
      std::lock_guard<std::mutex> lock(mutexJointCommands_);
      jointCommands.reset(new series_elastic_actuator_msgs::SeActuatorCommands(*jointCommands_));
    }
    nanoSecs = timer.getElapsedTimeNanoSec();
    if (nanoSecs > timeStep) {
      NODEWRAP_WARN("Lock and reset of joint commands took: %lf ms\n", (double)nanoSecs*1e-6);
    }




    {
      std::lock_guard<std::mutex> lockControllerManager(mutexModelAndControllerManager_);
      model_.getSeActuatorCommands(jointCommands);
    }
    nanoSecs = timer.getElapsedTimeNanoSec();
    if (nanoSecs > timeStep) {
      NODEWRAP_WARN("model_.getSeActuatorCommands(jointCommands) took: %lf ms\n", (double)nanoSecs*1e-6);
    }


    jointCommandsPublisher_.publish(boost::const_pointer_cast<const series_elastic_actuator_msgs::SeActuatorCommands>(jointCommands));
    nanoSecs = timer.getElapsedTimeNanoSec();
    if (nanoSecs > timeStep) {
      NODEWRAP_WARN("publish joint states: %lf ms\n", (double)nanoSecs*1e-6);
    }
  }

}


void LocomotionController::robotStateCallback(const quadruped_msgs::RobotState::ConstPtr& msg) {


//  std::lock_guard<std::mutex> lock(mutexRobotState_);
  {
    std::unique_lock<std::mutex> lock(mutexRobotState_);
    robotState_ = msg;
  }
  rcvdRobotState_.notify_all();
//  controllerWorker_.wake();

//  updateControllerAndPublish(msg);
}


bool LocomotionController::updateControllerWorker(const nodewrap::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, intermediate, end;

  {
    std::unique_lock<std::mutex> lock(mutexRobotState_);

    if ( !robotState_ || ( robotState_->header.stamp <= robotStateStamp_ ) ) {
      rcvdRobotState_.wait(lock);
    }

    model_.setRobotState(robotState_);
    robotStateStamp_ = robotState_->header.stamp;
  }

  //-- Start measuring computation time.
  start = std::chrono::steady_clock::now();
  controllerManager_.updateController();
  intermediate = std::chrono::steady_clock::now();
  publish();
  //---

  //-- Measure computation time.
  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();
  int64_t timeStep = (int64_t)(timeStep_*1e9);

  if (elapsedTimeNSecs > timeStep) {
    NODEWRAP_WARN("Computation of locomotion controller is not real-time! Elapsed time: %lf ms\n", (double)elapsedTimeNSecs*1e-6);

    int64_t advanceElapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(intermediate - start).count();
    NODEWRAP_WARN("Advance controller elapsed time: %lf ms\n", (double)advanceElapsedTimeNSecs*1e-6);
    int64_t publishElapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - intermediate).count();
    NODEWRAP_WARN("Publish elapsed time: %lf ms\n", (double)publishElapsedTimeNSecs*1e-6);
  }
  if (elapsedTimeNSecs > timeStep*10) {
      NODEWRAP_ERROR("Computation took more than 10 times the maximum allowed computation time (%lf ms)!", timeStep_*1e-3);
      int64_t advanceElapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(intermediate - start).count();
      NODEWRAP_WARN("Advance controller elapsed time: %lf ms\n", (double)advanceElapsedTimeNSecs*1e-6);
      int64_t publishElapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - intermediate).count();
      NODEWRAP_WARN("Publish elapsed time: %lf ms\n", (double)publishElapsedTimeNSecs*1e-6);

      std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
      controllerManager_.emergencyStop();
  }
  //---


  return true;
}


void LocomotionController::updateControllerAndPublish(const quadruped_msgs::RobotState::ConstPtr& robotState) {
  //-- Start measuring computation time.
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  start = std::chrono::steady_clock::now();
  //---
  std::lock_guard<std::mutex> lockUpdateControllerAndPublish(mutexUpdateControllerAndPublish_);

  NODEWRAP_DEBUG("Update locomotion controller.");

  {
    std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
    model_.setRobotState(robotState);
    controllerManager_.updateController();
  }
  publish();



  //-- Measure computation time.
  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();
  int64_t timeStep = (int64_t)(timeStep_*1e9);
  if (elapsedTimeNSecs > timeStep) {
    NODEWRAP_WARN("Computation of locomotion controller is not real-time! Elapsed time: %lf ms\n", (double)elapsedTimeNSecs*1e-6);
  }
  if (elapsedTimeNSecs > timeStep*10) {
      NODEWRAP_ERROR("Computation took more than 10 times the maximum allowed computation time (%lf ms)!", timeStep_*1e-3);
      std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
      controllerManager_.emergencyStop();
  }

  //---
}


void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutexJoystick_);
  std::lock_guard<std::mutex> lockControllerManager(mutexModelAndControllerManager_);

  ros::Duration age = (ros::Time::now()-msg->header.stamp);
  if (age >= ros::Duration(4.0)) {
    controllerManager_.emergencyStop();
    NODEWRAP_WARN("Joystick message is %lf seconds old! Called emergency stop!", age.toSec());
  }
  else {
    model_.setJoystickCommands(msg);


/*    // START + LF buttons
    if (msg->buttons[4] == 1 && msg->buttons[7] == 1 ) {
      locomotion_controller_msgs::SwitchController::Request  req;
      locomotion_controller_msgs::SwitchController::Response res;
      req.name = defaultController_;
      if(!controllerManager_.switchController(req,res)) {
      }
      ROS_INFO("Switched task by joystick (status: %d)",res.status);

    }*/


    // LT + LEFT JOY
    if (msg->buttons[4] == 1 && msg->axes[6] == 1 ) {
      locomotion_controller_msgs::SwitchController::Request  req;
      locomotion_controller_msgs::SwitchController::Response res;
      req.name = "LocoDemo";
      if(!controllerManager_.switchController(req,res)) {
      }
      ROS_INFO("Switched task by joystick to LocoDemo (status: %d)",res.status);

    }
    // LT + RIGHT JOY
    if (msg->buttons[4] == 1 && msg->axes[6] == -1 ) {
      locomotion_controller_msgs::SwitchController::Request  req;
      locomotion_controller_msgs::SwitchController::Response res;
      req.name = "Crawling";
      if(!controllerManager_.switchController(req,res)) {
      }
      ROS_INFO("Switched task by joystick to LocoCrawling (status: %d)",res.status);

    }



    // RB button
    if (msg->buttons[5] == 1 ) {
      NODEWRAP_WARN("Emergency stop by joystick!");
      controllerManager_.emergencyStop();
    }

    // LT + RT
    if (msg->axes[2] == -1 && msg->axes[5] == -1 ) {
      ROS_INFO("Resetting state estimator by joystick.");
      //---  Reset the estimator.
      if (resetStateEstimatorClient_.exists()) {
        ROS_INFO("Locomotion controller wants to reset state estimator.");
        locomotion_controller_msgs::ResetStateEstimator resetEstimatorService;
        resetEstimatorService.request.pose.orientation.w = 1.0;
        if(!resetStateEstimatorClient_.call(resetEstimatorService)) {
          ROS_ERROR("Locomotion controller could not reset state estimator.");
        }

      }
      else {
        ROS_ERROR("Service to reset estimator does not exist!");
      }
      //---
    }

  }
}

bool LocomotionController::emergencyStop(locomotion_controller_msgs::EmergencyStop::Request  &req,
                                         locomotion_controller_msgs::EmergencyStop::Response &res) {


  bool result = true;

  //--- Stop the controller.
  {
    std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
    if(!controllerManager_.emergencyStop()) {
      result = false;
    }
  }
  //---

// The estimator does not have to be reset when an emergency stop is invoked!!

/*
 //---  Reset the estimator.
 if (resetStateEstimatorClient_.exists()) {
   ROS_INFO("Locomotion controller wants to reset state estimator.");
   locomotion_controller_msgs::ResetStateEstimator resetEstimatorService;
   resetEstimatorService.request.pose.orientation.w = 1.0;
   if(!resetStateEstimatorClient_.call(resetEstimatorService)) {
     ROS_WARN("Locomotion controller could not reset state estimator.");
     result = false;
   }
 }
 //---
*/

  return result;
}

void LocomotionController::commandVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);

  // Ignore old messages for safety
  ros::Duration age = (ros::Time::now()-msg->header.stamp);
  if (age >= ros::Duration(2.0)) {
    ROS_WARN("Ignoring commanded velocity which is %lf seconds old. Commanded velocity was set to zero.", age.toSec());
    model_.setCommandVelocity(geometry_msgs::Twist());
  }
  else {
    model_.setCommandVelocity(msg->twist);
  }

}

void LocomotionController::mocapCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
  model_.setMocapData(msg);
}

void LocomotionController::seActuatorReadingsCallback(const series_elastic_actuator_msgs::SeActuatorReadings::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutexModelAndControllerManager_);
  model_.setSeActuatorReadings(msg);
}

} /* namespace locomotion_controller */
