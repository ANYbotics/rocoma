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
 * @file    LocomotionController.hpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */

#pragma once

//#define LC_ENABLE_LOGGER 1

#include <ros/ros.h>
#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include <roscpp_nodewrap/worker/Worker.h>

// ROS messages / services
#include <quadruped_msgs/QuadrupedState.h>
#include <sensor_msgs/Joy.h>
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <locomotion_controller_msgs/SwitchController.h>
#include <locomotion_controller_msgs/ResetStateEstimator.h>
#include <std_srvs/Empty.h>

#include <notification/NotificationPublisher.hpp>

#include "locomotion_controller/Model.hpp"
#include "locomotion_controller/ControllerManager.hpp"

#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/rotations/RotationDiffEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>


#include <boost/thread.hpp>
#include <boost/chrono.hpp>

namespace locomotion_controller {

class LocomotionController : public nodewrap::NodeImpl
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
  LocomotionController();
  virtual ~LocomotionController();

  void init();
  void cleanup();

  void updateControllerAndPublish();
  void getActuatorCommands(series_elastic_actuator_msgs::SeActuatorCommands& commands);

  template<class T>
  nodewrap::Worker addWrappedWorker(const std::string& name,
                                    const ros::Rate& defaultRate,
                                    bool (T::*fp)(const nodewrap::WorkerEvent&));

  nodewrap::Worker addWrappedWorker(const std::string& name,
                                    const nodewrap::WorkerOptions& defaultOptions);

  double getLoggerSamplingWindow() const;
  double getLoggerSamplingFrequency() const;


  void setQuadrupedState(const quadruped_msgs::QuadrupedState& msg);
  void setActuatorReadings(const series_elastic_actuator_msgs::SeActuatorReadings& msg);
 protected:
  void initializeMessages();
  void initializeServices();
  void initializePublishers();
  void initializeSubscribers();

  void publish();

  void quadrupedStateCallback(const quadruped_msgs::QuadrupedState::ConstPtr& msg);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
  bool emergencyStop(locomotion_controller_msgs::EmergencyStop::Request  &req,
                     locomotion_controller_msgs::EmergencyStop::Response &res);
  void velocityCommandsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void mocapCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void actuatorCommandsSubscriberConnect(const ros::SingleSubscriberPublisher& pub);
  void actuatorCommandsSubscriberDisconnect(const ros::SingleSubscriberPublisher& pub);
  void seActuatorReadingsCallback(const series_elastic_actuator_msgs::SeActuatorReadings::ConstPtr& msg);

  const std::string& getQuadrupedName() const;

  /*
   * Worker callbacks
   */
  bool updateControllerWorker(const nodewrap::WorkerEvent& event);

  void update();
  void updateQuadrupedState();
  void updateJoystickReadings();
  void updateVelocityCommands();
  void updateActuatorReadings();
  void updateActuatorCommands();

 private:
  bool loadQuadrupedModelFromFile_;
  bool useWorker_;
  bool subscribeToQuadrupedState_;
  bool subscribeToActuatorReadings_;
  double timeStep_;
  bool isRealRobot_;
  double loggerSamplingWindow_;
  double loggerSamplingFrequency_;
  std::string defaultController_;
  std::string quadrupedName_;

  model::Model model_;
  std::mutex mutexModel_;

  ControllerManager controllerManager_;

  ros::Subscriber quadrupedStateSubscriber_;
  ros::Subscriber joystickSubscriber_;
  ros::Subscriber velocityCommandsSubscriber_;

  // this is only temporary:
  ros::Subscriber mocapSubscriber_;
  ros::Subscriber seActuatorReadingsSubscriber_;

  ros::Publisher actuatorommandsPublisher_;

  std::shared_ptr<notification::NotificationPublisher> notificationPublisher_;


  ros::ServiceServer switchControllerService_;
  ros::ServiceServer emergencyStopService_;
  ros::ServiceServer getAvailableControllersService_;
  ros::ServiceServer getActiveControllerService_;
  ros::ServiceClient resetStateEstimatorClient_;

  series_elastic_actuator_msgs::SeActuatorCommandsPtr actuatorCommands_;
  boost::shared_mutex mutexActuatorCommands_;

  quadruped_msgs::QuadrupedStatePtr quadrupedState_;
  boost::shared_mutex mutexQuadrupedState_;

  series_elastic_actuator_msgs::SeActuatorReadingsPtr actuatorReadings_;
  boost::shared_mutex mutexActuatorReadings_;

  sensor_msgs::JoyPtr joystickReadings_;
  boost::shared_mutex mutexJoystickReadings_;

  geometry_msgs::TwistStampedPtr velocityCommands_;
  boost::shared_mutex mutexVelocityCommands_;


  /*
   * Nodewrap worker
   */
  nodewrap::Worker controllerWorker_;
  boost::condition_variable_any rcvdQuadrupedState_;
  ros::Time quadrupedStateStamp_;

  size_t actuatorCommandsNumSubscribers_;


};

} /* namespace locomotion_controller */
