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

#include <ros/ros.h>
#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

#include <quadruped_msgs/QuadrupedState.h>
#include <sensor_msgs/Joy.h>
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include "locomotion_controller/Model.hpp"
#include "locomotion_controller/ControllerManager.hpp"


#include <kindr/rotations/RotationEigen.hpp>


#include <kindr/rotations/RotationDiffEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>


#include <memory>
#include <mutex>
#include <condition_variable>


#include <roscpp_nodewrap/worker/Worker.h>


#define LC_ENABLE_LOGGER 1

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

  template<class T>
  nodewrap::Worker addWrappedWorker(const std::string& name,
                                    const ros::Rate& defaultRate,
                                    bool (T::*fp)(const nodewrap::WorkerEvent&))
  {
    return this->addWorker<T>(name, defaultRate, fp);
  }

  nodewrap::Worker addWrappedWorker(const std::string& name,
                                    const nodewrap::WorkerOptions& defaultOptions);

  double getLoggerSamplingWindow() const;
  double getLoggerSamplingFrequency() const;


 protected:
  void publish();
  void quadrupedStateCallback(const quadruped_msgs::QuadrupedState::ConstPtr& msg);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
  bool emergencyStop(locomotion_controller_msgs::EmergencyStop::Request  &req,
                     locomotion_controller_msgs::EmergencyStop::Response &res);
  void commandVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void mocapCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void seActuatorReadingsCallback(const series_elastic_actuator_msgs::SeActuatorReadings::ConstPtr& msg);

  void initializeMessages();
  void initializeServices();
  void initializePublishers();
  void initializeSubscribers();

  void updateControllerAndPublish(const quadruped_msgs::QuadrupedState::ConstPtr& quadrupedState);
  const std::string& getQuadrupedName() const;

  /*
   * Worker callbacks
   */
  bool updateControllerWorker(const nodewrap::WorkerEvent& event);

 private:
  bool loadQuadrupedModelFromFile_;
  bool useWorker_;
  double timeStep_;
  bool isRealRobot_;
  double loggerSamplingWindow_;
  double loggerSamplingFrequency_;
  std::string defaultController_;
  std::string quadrupedName_;

  model::Model model_;
  ControllerManager controllerManager_;

  ros::Subscriber quadrupedStateSubscriber_;
  ros::Subscriber joystickSubscriber_;
  ros::Subscriber commandVelocitySubscriber_;

  // this is only temporary:
  ros::Subscriber mocapSubscriber_;
  ros::Subscriber seActuatorReadingsSubscriber_;

  ros::Publisher jointCommandsPublisher_;
  ros::ServiceServer switchControllerService_;
  ros::ServiceServer emergencyStopService_;
  ros::ServiceServer getAvailableControllersService_;
  ros::ServiceServer getActiveControllerService_;
  ros::ServiceClient resetStateEstimatorClient_;

  series_elastic_actuator_msgs::SeActuatorCommandsPtr jointCommands_;

  quadruped_msgs::QuadrupedStateConstPtr quadrupedState_;

//   std::mutex mutexControllerManager_;
  std::mutex mutexJointCommands_;
  std::mutex mutexJoystick_;
  std::mutex mutexModel_;
  std::mutex mutexUpdateControllerAndPublish_;
  std::mutex mutexQuadrupedState_;

  /*
   * Nodewrap worker
   */
  nodewrap::Worker controllerWorker_;
  std::condition_variable rcvdQuadrupedState_;
  ros::Time quadrupedStateStamp_;

  std::shared_ptr<ros::CallbackQueue> jointCommandsCallbackQueue_;
  size_t jointCommandsNumSubscribers_;
  void jointCommandsSubscriberConnect(const ros::SingleSubscriberPublisher& pub);
  void jointCommandsSubscriberDisconnect(const ros::SingleSubscriberPublisher& pub);

};

} /* namespace locomotion_controller */
