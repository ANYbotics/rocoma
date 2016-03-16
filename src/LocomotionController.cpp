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

#include <quadruped_model/robots/quadrupeds.hpp>
#include <quadruped_model/robots/starleth.hpp>
#include <quadruped_model/robots/anymal.hpp>
#include <quadruped_assembly/quadrupeds.hpp>

#include <signal_logger/logger.hpp>
#include <signal_logger/LoggerNone.hpp>
#include <signal_logger_std/LoggerStd.hpp>
#include <signal_logger_ros/LoggerRos.hpp>

#include <parameter_handler/parameter_handler.hpp>
#include <parameter_handler_ros/ParameterHandlerRos.hpp>


#include <chrono>
#include <cstdint>
#include <string>

#define NOTIFICATION_ID "LMC"

NODEWRAP_EXPORT_CLASS(locomotion_controller, locomotion_controller::LocomotionController)

namespace locomotion_controller {

LocomotionController::LocomotionController():
    loadQuadrupedModelFromFile_(false),
    useWorker_(true),
    subscribeToQuadrupedState_(true),
    subscribeToActuatorReadings_(true),
    timeStep_(0.0025),
    isRealRobot_(true),
    loggerSamplingWindow_(60.0),
    loggerSamplingFrequency_(1.0),
    model_(),
    controllerManager_(this),
    defaultController_("LocoDemo"),
    quadrupedName_("starleth"),
    actuatorCommandsNumSubscribers_(0)
{


}

LocomotionController::~LocomotionController()
{
}

void LocomotionController::init() {
  //--- Read parameters.
  getNodeHandle().param<double>("controller/time_step", timeStep_, 0.0025);
  bool simulation = false;
  getNodeHandle().param<bool>("controller/simulation", simulation, false);
  isRealRobot_ = !simulation;
  getNodeHandle().param<bool>("controller/use_worker", useWorker_, true);
  getNodeHandle().param<bool>("controller/subscribe_state", subscribeToQuadrupedState_, true);
  getNodeHandle().param<bool>("controller/subscribe_actuator_readings", subscribeToActuatorReadings_, true);

  getNodeHandle().param<std::string>("controller/default", defaultController_, "LocoDemo");
  getNodeHandle().param<std::string>("quadruped/name", quadrupedName_, "starleth");

  ROS_INFO_STREAM("Is controlling a real robot: " << (isRealRobot_ ? "yes": "no"));


#ifdef LC_ENABLE_LOGGER
  //--- Configure logger.
  std::string loggingScriptFilename;
  getNodeHandle().param<std::string>("logger/script", loggingScriptFilename, "");
  if (loggingScriptFilename.empty()){
    loggingScriptFilename = ros::package::getPath("locomotion_controller") + std::string{"/config/logging.script"};
  }
  getNodeHandle().param<double>("logger/sampling_window", loggerSamplingWindow_, 60.0);
  getNodeHandle().param<double>("logger/sampling_frequency", loggerSamplingFrequency_, 1.0);

  std::string loggerClass;
  getNodeHandle().param<std::string>("logger/class", loggerClass, "std");

  if (loggerClass.compare("ros") == 0) {
    NODEWRAP_INFO("[LocomotionController::init] Logger type: ros");
    // initialize ros logger
    signal_logger::logger.reset(new signal_logger_ros::LoggerRos(getNodeHandle()));
  } else if (loggerClass.compare("std") == 0) {
    NODEWRAP_INFO("[LocomotionController::init] Logger type: std");
    // initialize std logger as fallback logger
    signal_logger::logger.reset(new signal_logger_std::LoggerStd());
    signal_logger_std::LoggerStd* loggerStd = static_cast<signal_logger_std::LoggerStd*>(signal_logger::logger.get());
    loggerStd->setVerboseLevel(signal_logger_std::LoggerStd::VL_DEBUG);
  } else {
    NODEWRAP_INFO("[LocomotionController::init] Logger type: none");
    signal_logger::logger.reset(new signal_logger::LoggerNone());
  }

  signal_logger::logger->initLogger(static_cast<int>(1.0/timeStep_), static_cast<int>(loggerSamplingFrequency_), static_cast<int>(loggerSamplingWindow_), loggingScriptFilename);
  NODEWRAP_INFO("[LocomotionController::init] Initialize logger with sampling window: %4.2fs, "
                "sampling frequency: %dHz and script: %s.", signal_logger::logger->getSamplingWindow(), signal_logger::logger->getSamplingFrequency(), loggingScriptFilename.c_str());
  //---
#endif

  //--- Configure parameter handler
  parameter_handler::handler.reset(new parameter_handler_ros::ParameterHandlerRos());
  parameter_handler_ros::ParameterHandlerRos* parameterHandlerRos = static_cast<parameter_handler_ros::ParameterHandlerRos*>(parameter_handler::handler.get());
  parameterHandlerRos->setNodeHandle(getNodeHandle());
  parameterHandlerRos->initializeServices();
  //---

  //--- Configure controllers
  {
    std::lock_guard<std::mutex> lockModel(mutexModel_);

    quadruped_model::Quadrupeds quadrupedEnum;
    if (quadrupedName_ == "starleth") quadrupedEnum = quadruped_model::Quadrupeds::StarlETH;
    if (quadrupedName_ == "anymal") quadrupedEnum = quadruped_model::Quadrupeds::Anymal;


    try {
      if (loadQuadrupedModelFromFile_) {
        std::string quadrupedSetup;
        getNodeHandle().param<std::string>("quadruped/setup", quadrupedSetup, "minimal");
        std::string urdfModelFile = ros::package::getPath("quadruped_model") + "/resources/" + quadrupedName_ + "_" + quadrupedSetup + ".urdf";
        NODEWRAP_INFO("[Locomotion Controller] Initializing model for %s with %s setup.", quadrupedName_.c_str(), quadrupedSetup.c_str());
        model_.initializeForControllerFromFile(timeStep_,isRealRobot_, urdfModelFile, quadrupedEnum);
      }
      else {
        std::string quadrupedUrdfDescription;
        getNodeHandle().param<std::string>("/quadruped_description", quadrupedUrdfDescription, "");
        NODEWRAP_INFO("[Locomotion Controller] Initializing model for %s.", quadrupedName_.c_str());
        model_.initializeForController(timeStep_,isRealRobot_, quadrupedUrdfDescription, quadrupedEnum);
      }
    } catch (std::exception& ex) {
      ROS_FATAL_STREAM("[Locomotion Controller] Could not initialize the model: " << ex.what());
      return;
    }


//    model_.getRobotModel()->params().printParams();
    model_.addVariablesToLog();


    controllerManager_.setIsRealRobot(isRealRobot_);
    controllerManager_.setupControllers(timeStep_, model_.getState(), model_.getCommand(), model_.getStateMutex(), model_.getCommandMutex(), getNodeHandle());
  }
  //---

  initializeMessages();
  initializeServices();
  initializePublishers();
  initializeSubscribers();

  //-- Addd variables
  std::string nsPrefix{"/sea/"};
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexActuatorReadings_);

   for(int i=0; i<actuatorReadings_->readings.size(); i++) {
     const std::string name = actuatorReadings_->readings[i].state.name;
     std::string ns = nsPrefix + name + "/state/";
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].state.jointPosition, "jointPosition", ns, "rad");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].state.jointVelocity, "jointVelocity", ns, "rad/s");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].state.actuatorPosition, "actuatorPosition", ns, "rad");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].state.actuatorVelocity, "actuatorVelocity", ns, "rad/s");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].state.torque, "torque", ns, "Nm");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].state.current, "current", ns, "A");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].state.statusword, "statusword", ns, "A");
     ns = nsPrefix + name + "/commanded/";
     signal_logger::logger->addIntToLog(actuatorReadings_->readings[i].commanded.mode, "mode", ns, "-");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].commanded.jointPosition, "jointPosition", ns, "rad");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].commanded.jointVelocity, "jointVelocity", ns, "rad/s");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].commanded.actuatorPosition, "actuatorPosition", ns, "rad");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].commanded.actuatorVelocity, "actuatorVelocity", ns, "rad/s");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].commanded.torque, "torque", ns, "Nm");
     signal_logger::logger->addDoubleToLog(actuatorReadings_->readings[i].commanded.current, "current", ns, "A");
   }
   signal_logger::logger->updateLogger(true);

  }

  /*
   * Start workers
   */
  nodewrap::WorkerOptions workerOptions;
  workerOptions.callback = boost::bind(&LocomotionController::updateControllerWorker, this, _1);
  workerOptions.frequency = 1.0/timeStep_;
  workerOptions.autostart = useWorker_;
  workerOptions.synchronous = false;
  workerOptions.privateCallbackQueue = true;
  workerOptions.priority = 99;
  controllerWorker_ = addWorker("controller", workerOptions);

  notificationPublisher_->notify(notification::Level::LEVEL_INFO,
                             std::string{"Starting up."},
                             std::string{"Locomotion controller is starting up."},
                             NOTIFICATION_ID);
}


void LocomotionController::cleanup() {
  notificationPublisher_->notify(notification::Level::LEVEL_ERROR,
                             std::string{"LMC is shutting down."},
                             std::string{"Locomotion controller is shutting down."},
                             NOTIFICATION_ID);
  notificationPublisher_->publish();

 controllerWorker_.cancel(true);

 NODEWRAP_INFO("Cleaning up locomotion controller.");
 controllerManager_.cleanup();


}


const std::string& LocomotionController::getQuadrupedName() const {
  return quadrupedName_;
}


nodewrap::Worker LocomotionController::addWrappedWorker(
    const std::string& name, const nodewrap::WorkerOptions& defaultOptions)
{
  return this->addWorker(name, defaultOptions);
}

template<class T>
nodewrap::Worker LocomotionController::addWrappedWorker(const std::string& name,
                                  const ros::Rate& defaultRate,
                                  bool (T::*fp)(const nodewrap::WorkerEvent&))
{
  return this->addWorker<T>(name, defaultRate, fp);
}

double LocomotionController::getLoggerSamplingWindow() const {
  return loggerSamplingWindow_;
}

double LocomotionController::getLoggerSamplingFrequency() const {
  return loggerSamplingFrequency_;
}

void LocomotionController::initializeMessages() {

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexQuadrupedState_);
    quadrupedState_.reset(new quadruped_msgs::QuadrupedState);
    quadruped_description::initializeQuadrupedState(*quadrupedState_);
  }

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexActuatorCommands_);
    actuatorCommands_.reset(new series_elastic_actuator_msgs::SeActuatorCommands);
    quadruped_description::initializeSeActuatorCommands(*actuatorCommands_);
  }

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexActuatorReadings_);
    actuatorReadings_.reset(new series_elastic_actuator_msgs::SeActuatorReadings);
    quadruped_description::initializeSeActuatorReadings(*actuatorReadings_);
  }

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
    joystickReadings_.reset(new sensor_msgs::Joy);
    joystickReadings_->header.stamp = ros::Time::now();
    joystickReadings_->axes.resize(7, 0.0);
    joystickReadings_->buttons.resize(15, 0.0);
  }

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
    velocityCommands_.reset(new geometry_msgs::TwistStamped);
    velocityCommands_->header.stamp = ros::Time::now();
  }

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
  actuatorCommandsNumSubscribers_ = 0;
  ros::AdvertiseOptions actuatorCommandsOptions;
  actuatorCommandsOptions.init<series_elastic_actuator_msgs::SeActuatorCommands>("/command_seactuators",
                                                                              100,
                                                                              boost::bind(&LocomotionController::actuatorCommandsSubscriberConnect,this,_1),
                                                                              boost::bind(&LocomotionController::actuatorCommandsSubscriberDisconnect,this,_1));

  actuatorommandsPublisher_ = advertise("command_seactuators", actuatorCommandsOptions);
  ROS_INFO_STREAM("Commands topic name: " << actuatorommandsPublisher_.getTopic());
  /*****************************/

  notificationPublisher_.reset(new notification::NotificationPublisher("default", getNodeHandle(), false));

}

void LocomotionController::initializeSubscribers() {
  joystickSubscriber_ = subscribe("joy", "/joy", 10, &LocomotionController::joystickCallback, ros::TransportHints().tcpNoDelay());
  velocityCommandsSubscriber_ = subscribe("command_velocity", "/command_velocity", 100, &LocomotionController::velocityCommandsCallback, ros::TransportHints().tcpNoDelay());
  //--- temporary
  mocapSubscriber_ = subscribe("mocap", "mocap", 100, &LocomotionController::mocapCallback, ros::TransportHints().tcpNoDelay());
  //---

  if (subscribeToActuatorReadings_) {
    seActuatorReadingsSubscriber_ = subscribe("actuator_readings", "/actuator_readings", 100, &LocomotionController::seActuatorReadingsCallback, ros::TransportHints().tcpNoDelay());
  }

  if (subscribeToQuadrupedState_) {
    // this should be last since it will start the controller loop
    quadrupedStateSubscriber_ = subscribe("quadruped_state", "/robot", 100, &LocomotionController::quadrupedStateCallback, ros::TransportHints().tcpNoDelay());
  }

}


void LocomotionController::actuatorCommandsSubscriberConnect(const ros::SingleSubscriberPublisher& pub) {
  actuatorCommandsNumSubscribers_++;
  ROS_INFO_STREAM("[LocomotionController] Increasing number of subscribers to commands to: " << actuatorCommandsNumSubscribers_);
}
void LocomotionController::actuatorCommandsSubscriberDisconnect(const ros::SingleSubscriberPublisher& pub) {
  actuatorCommandsNumSubscribers_--;
  ROS_INFO_STREAM("[LocomotionController] Decreasing number of subscribers to commands to: " << actuatorCommandsNumSubscribers_);
}


void LocomotionController::publish()  {
  if (actuatorCommandsNumSubscribers_ > 0u) {
    series_elastic_actuator_msgs::SeActuatorCommandsPtr actuatorCommands;
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexActuatorCommands_);
      actuatorCommands.reset(new series_elastic_actuator_msgs::SeActuatorCommands(*actuatorCommands_));
    }
    actuatorommandsPublisher_.publish(boost::const_pointer_cast<const series_elastic_actuator_msgs::SeActuatorCommands>(actuatorCommands));
  }

  notificationPublisher_->publish();
}

void LocomotionController::updateActuatorCommands()
{
  boost::unique_lock<boost::shared_mutex> lock(mutexActuatorCommands_);
  boost::lock_guard<std::mutex> lockModel(mutexModel_);
  model_.getSeActuatorCommands(actuatorCommands_);
}


void LocomotionController::setQuadrupedState(const quadruped_msgs::QuadrupedState& msg) {
  boost::unique_lock<boost::shared_mutex> lock(mutexQuadrupedState_);
  *quadrupedState_ = msg;
}

void LocomotionController::setActuatorReadings(const series_elastic_actuator_msgs::SeActuatorReadings& msg) {
  boost::unique_lock<boost::shared_mutex> lock(mutexActuatorReadings_);

  // Note that the std::string variables cannot be copied because they cannot be stored in shared memory.
  for (int i=0; i<actuatorReadings_->readings.size(); i++) {
    actuatorReadings_->readings[i].state.header.stamp =  msg.readings[i].state.header.stamp;
    actuatorReadings_->readings[i].state.jointPosition = msg.readings[i].state.jointPosition;
    actuatorReadings_->readings[i].state.jointVelocity = msg.readings[i].state.jointVelocity;
    actuatorReadings_->readings[i].state.actuatorPosition = msg.readings[i].state.actuatorPosition;
    actuatorReadings_->readings[i].state.actuatorVelocity = msg.readings[i].state.actuatorVelocity;
    actuatorReadings_->readings[i].state.torque = msg.readings[i].state.torque;
    actuatorReadings_->readings[i].state.current = msg.readings[i].state.current;
    actuatorReadings_->readings[i].state.statusword = msg.readings[i].state.statusword;

    actuatorReadings_->readings[i].commanded.header.stamp = msg.readings[i].commanded.header.stamp;
    actuatorReadings_->readings[i].commanded.mode = msg.readings[i].commanded.mode;
    actuatorReadings_->readings[i].commanded.actuatorPosition = msg.readings[i].commanded.actuatorPosition;
    actuatorReadings_->readings[i].commanded.actuatorVelocity = msg.readings[i].commanded.actuatorVelocity;
    actuatorReadings_->readings[i].commanded.jointPosition = msg.readings[i].commanded.jointPosition;
    actuatorReadings_->readings[i].commanded.jointVelocity = msg.readings[i].commanded.jointVelocity;
    actuatorReadings_->readings[i].commanded.torque = msg.readings[i].commanded.torque;
    actuatorReadings_->readings[i].commanded.current = msg.readings[i].commanded.current;
  }

}

void LocomotionController::quadrupedStateCallback(const quadruped_msgs::QuadrupedState::ConstPtr& msg) {
  {
    boost::unique_lock<boost::shared_mutex> lock(mutexQuadrupedState_);
    *quadrupedState_ = *msg;
  }
  
  rcvdQuadrupedState_.notify_all();

  if (!useWorker_) {
    updateControllerAndPublish();
  }
}


bool LocomotionController::updateControllerWorker(const nodewrap::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  
  {
    bool quadrupedStateOk = false;
    
    boost::unique_lock<boost::shared_mutex> lockQuadrupedState(mutexQuadrupedState_);

    // This indicates that we have never received a robot state, thus we just return
    if ( !quadrupedState_ )
      return true;

    if (isRealRobot_) {
      // real robot
      if ( quadrupedState_->header.stamp <= quadrupedStateStamp_ ) {
        boost::chrono::nanoseconds quadrupedStateTimeoutNSecs((int64_t(10*timeStep_*1e9)));
        //ROS_INFO_STREAM("prev: quadrupedStateStamp: " << quadrupedStateStamp_ << " current:" << quadrupedState_->header.stamp );
        if (rcvdQuadrupedState_.wait_for(lockQuadrupedState, quadrupedStateTimeoutNSecs) == boost::cv_status::no_timeout) {
          //ROS_WARN_STREAM("Locomotion Controller: Timing might be an issue!");
          quadrupedStateOk = true;
        }
      }
      else {
        quadrupedStateOk = true;
      }

    } else {
      // simulated robot
      ros::Time startTime = ros::Time::now();
      int cycleCounter = 0;
      while (cycleCounter < 100) {
        if (quadrupedState_->header.stamp >= quadrupedStateStamp_) {
          quadrupedStateOk = true;
          break;
        }
        mutexQuadrupedState_.unlock();
        ros::Time::sleepUntil(startTime + ros::Duration(cycleCounter*timeStep_/10.0));
        mutexQuadrupedState_.lock();
        cycleCounter++;
      }

    }


    if (quadrupedStateOk) {
      quadrupedStateStamp_ = quadrupedState_->header.stamp;
    }
    else {
      NODEWRAP_ERROR("Robot state update was not received within 10 times the maximum allowed computation time (%lf ms)!", 10*timeStep_*1e3);
      controllerManager_.emergencyStop();
    }
  }
  

  //-- Start measuring computation time.
  start = std::chrono::steady_clock::now();
  
  update();
  publish();


  //-- Measure computation time.
  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();
  const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
  const int64_t maxComputationTimeNSecs = timeStepNSecs*10.0;

  if (elapsedTimeNSecs > timeStepNSecs) {
    if (isRealRobot_) {
      NODEWRAP_WARN("Computation of locomotion controller is not real-time! Elapsed time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
    else {
      NODEWRAP_WARN_THROTTLE(3.0, "Computation of locomotion controller is not real-time! Elapsed time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
  }
  if (isRealRobot_ && (elapsedTimeNSecs > maxComputationTimeNSecs)) {
    NODEWRAP_ERROR("Computation took more than 10 times the maximum allowed computation time (%lf ms > %lf ms)!", (double)elapsedTimeNSecs*1e-6, (double)maxComputationTimeNSecs*1e-6);
    controllerManager_.emergencyStop();
  }
  //---

  return true;
}


void LocomotionController::updateControllerAndPublish() {
  //-- Start measuring computation time.
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  start = std::chrono::steady_clock::now();
  //---
  
  NODEWRAP_DEBUG("Update locomotion controller.");

  update();
  publish();

  //-- Measure computation time.
  end = std::chrono::steady_clock::now();
  int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();
  const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
  const int64_t maxComputationTimeNSecs = timeStepNSecs*10.0;

  if (elapsedTimeNSecs > timeStepNSecs) {
    if (isRealRobot_) {
      NODEWRAP_WARN("Computation of locomotion controller is not real-time! Elapsed time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
    else {
      NODEWRAP_WARN_THROTTLE(3.0, "Computation of locomotion controller is not real-time! Elapsed time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
  }
  if (isRealRobot_ && (elapsedTimeNSecs > maxComputationTimeNSecs)) {
    NODEWRAP_ERROR("Computation took more than 10 times the maximum allowed computation time (%lf ms > %lf ms)!", (double)elapsedTimeNSecs*1e-6, (double)maxComputationTimeNSecs*1e-6);
    controllerManager_.emergencyStop();
  }
  //---
}

void LocomotionController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
  *joystickReadings_ = *msg;
}


bool LocomotionController::emergencyStop(locomotion_controller_msgs::EmergencyStop::Request  &req,
                                         locomotion_controller_msgs::EmergencyStop::Response &res) {


  bool result = true;

  //--- Stop the controller.
  if(!controllerManager_.emergencyStop()) {
    result = false;
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

void LocomotionController::velocityCommandsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // Ignore old messages for safety
  ros::Duration age = (ros::Time::now()-msg->header.stamp);
  if (age >= ros::Duration(2.0)) {
    boost::unique_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
    velocityCommands_->header.stamp = ros::Time::now();
    velocityCommands_->twist.linear.x = 0.0;
    velocityCommands_->twist.linear.y = 0.0;
    velocityCommands_->twist.linear.z = 0.0;
    velocityCommands_->twist.angular.x = 0.0;
    velocityCommands_->twist.angular.y = 0.0;
    velocityCommands_->twist.angular.z = 0.0;
    ROS_WARN("Ignoring commanded velocity which is %lf seconds old. Commanded velocity was set to zero.", age.toSec());
  }
  else {
    boost::unique_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
    *velocityCommands_ = *msg;
  }

}

void LocomotionController::mocapCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lockModel(mutexModel_);
  model_.setMocapData(msg);
}

void LocomotionController::seActuatorReadingsCallback(const series_elastic_actuator_msgs::SeActuatorReadings::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(mutexActuatorReadings_);
  *actuatorReadings_ = *msg;
}

void LocomotionController::updateActuatorReadings() {
  std::lock_guard<std::mutex> lockModel(mutexModel_);
  boost::shared_lock<boost::shared_mutex> lock(mutexActuatorReadings_);
  model_.setSeActuatorReadings(actuatorReadings_);
}

void LocomotionController::getActuatorCommands(series_elastic_actuator_msgs::SeActuatorCommands& commands) {
  boost::shared_lock<boost::shared_mutex> lock(mutexActuatorCommands_);
//  commands = *actuatorCommands_;
  for (int i=0; i<12; i++) {
    commands.commands[i].header.stamp = actuatorCommands_->commands[i].header.stamp;
    commands.commands[i].mode = actuatorCommands_->commands[i].mode;
    commands.commands[i].actuatorPosition = actuatorCommands_->commands[i].actuatorPosition;
    commands.commands[i].actuatorVelocity = actuatorCommands_->commands[i].actuatorVelocity;
    commands.commands[i].jointPosition = actuatorCommands_->commands[i].jointPosition;
    commands.commands[i].jointVelocity = actuatorCommands_->commands[i].jointVelocity;
    commands.commands[i].torque = actuatorCommands_->commands[i].torque;
    commands.commands[i].current = actuatorCommands_->commands[i].current;
  }
}

void LocomotionController::update() {
  updateQuadrupedState();
  updateJoystickReadings();
  updateVelocityCommands();
  updateActuatorReadings();
  controllerManager_.updateController();
  updateActuatorCommands();
}

void LocomotionController::updateJoystickReadings() {


  /*
   * Ignore too old joystick commands.
   */
  ros::Duration age;
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
    age = (ros::Time::now()-joystickReadings_->header.stamp);
  }

  if (isRealRobot_ && (age >= ros::Duration(4.0)) ) {
    NODEWRAP_WARN("Joystick message is %lf seconds old! Called emergency stop!", age.toSec());
    controllerManager_.emergencyStop();
    {
      boost::unique_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
      joystickReadings_->header.stamp = ros::Time::now();
    }
  }
  else {
    boost::shared_lock<boost::shared_mutex> lock(mutexJoystickReadings_);

    //-- update the model
    {
      std::lock_guard<std::mutex> lockModel(mutexModel_);
      model_.setJoystickCommands(*joystickReadings_);
    }
    //--

    // LT + LEFT JOY
    if (joystickReadings_->buttons[4] == 1 && joystickReadings_->axes[6] == 1 ) {
      locomotion_controller_msgs::SwitchController::Request  req;
      locomotion_controller_msgs::SwitchController::Response res;
      req.name = "loco_demo_ros";
      if(controllerManager_.switchController(req,res)) {
        NODEWRAP_INFO("Switched task by joystick to loco_demo_ros (status: %d)", res.status);
      }
    }
    // LT + RIGHT JOY
    if (joystickReadings_->buttons[4] == 1 && joystickReadings_->axes[6] == -1 ) {
      locomotion_controller_msgs::SwitchController::Request  req;
      locomotion_controller_msgs::SwitchController::Response res;
      req.name = "loco_crawling_ros";
      if(controllerManager_.switchController(req,res)) {
        NODEWRAP_INFO("Switched task by joystick to LocoCrawling (status: %d)",res.status);
      }
    }

    // RB button
    if (joystickReadings_->buttons[5] == 1 ) {
      NODEWRAP_WARN("Emergency stop by joystick!");
      controllerManager_.emergencyStop();
    }

    // LT + RT
    if (joystickReadings_->axes[2] == -1 && joystickReadings_->axes[5] == -1 ) {
      NODEWRAP_INFO("Resetting state estimator by joystick.");
      //---  Reset the estimator.
      if (resetStateEstimatorClient_.exists()) {
        NODEWRAP_INFO("Locomotion controller wants to reset state estimator.");
        locomotion_controller_msgs::ResetStateEstimator resetEstimatorService;
        resetEstimatorService.request.pose.orientation.w = 1.0;
        if(!resetStateEstimatorClient_.call(resetEstimatorService)) {
          NODEWRAP_ERROR("Locomotion controller could not reset state estimator.");
        }
      }
      else {
        NODEWRAP_ERROR("Service to reset estimator does not exist!");
      }
      //---
    }

  }
}

void LocomotionController::updateVelocityCommands() {
  boost::shared_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
  std::lock_guard<std::mutex> lockModel(mutexModel_);
  model_.setCommandVelocity(velocityCommands_->twist);
}

void LocomotionController::updateQuadrupedState() {
  boost::shared_lock<boost::shared_mutex> lock(mutexQuadrupedState_);
  std::lock_guard<std::mutex> lockModel(mutexModel_);
  model_.setQuadrupedState(quadrupedState_);
}

} /* namespace locomotion_controller */
