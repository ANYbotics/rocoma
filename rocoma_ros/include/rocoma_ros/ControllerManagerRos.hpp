#pragma once

// rocoma
#include "rocoma/ControllerManager.hpp"
#include "rocoma/controllers/ControllerAdapter.hpp"

// rocoma plugin
#include "rocoma_plugin/interfaces/ControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/EmergencyControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/FailproofControllerPluginInterface.hpp"

// rocoma msgs
#include "rocoma_msgs/GetAvailableControllers.h"
#include "rocoma_msgs/EmergencyStop.h"
#include "rocoma_msgs/SwitchController.h"
#include "rocoma_msgs/GetActiveController.h"

// any msgs
#include "any_msgs/State.h"

// roco
#include "roco/controllers/adapters/ControllerAdapterInterface.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// ros
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/console.h>

// stl
#include <string>
#include <vector>
#include <map>
#include <memory>


namespace rocoma_ros {

template<typename State_, typename Command_>
class ControllerManagerRos : public rocoma::ControllerManager {

 public:
  ControllerManagerRos(ros::NodeHandle& nodeHandle,
                       const std::string & scopedStateName,
                       const std::string & scopedCommandName);
  ~ControllerManagerRos();

  bool setupControllerPair(const std::string & controllerPluginName,
                           const std::string & emgcyControllerPluginName,
                           std::shared_ptr<State_> state,
                           std::shared_ptr<Command_> command,
                           std::shared_ptr<boost::shared_mutex> mutexState,
                           std::shared_ptr<boost::shared_mutex> mutexCommand,
                           std::shared_ptr<any_worker::WorkerManager> workerManager);

  bool setupFailproofController(const std::string & controllerPluginName,
                                std::shared_ptr<State_> state,
                                std::shared_ptr<Command_> command,
                                std::shared_ptr<boost::shared_mutex> mutexState,
                                std::shared_ptr<boost::shared_mutex> mutexCommand,
                                std::shared_ptr<any_worker::WorkerManager> workerManager);

  bool setupControllers(const std::string & failproofControllerName,
                        const std::vector< std::pair<std::string, std::string> > & controllerNameMap,
                        std::shared_ptr<State_> state,
                        std::shared_ptr<Command_> command,
                        std::shared_ptr<boost::shared_mutex> mutexState,
                        std::shared_ptr<boost::shared_mutex> mutexCommand,
                        std::shared_ptr<any_worker::WorkerManager> workerManager);

  bool emergencyStop(rocoma_msgs::EmergencyStop::Request  &req,
                     rocoma_msgs::EmergencyStop::Response &res);

  bool switchController(rocoma_msgs::SwitchController::Request  &req,
                        rocoma_msgs::SwitchController::Response &res);

  bool getAvailableControllers(rocoma_msgs::GetAvailableControllers::Request &req,
                               rocoma_msgs::GetAvailableControllers::Response &res);

  bool getActiveController(rocoma_msgs::GetActiveController::Request &req,
                           rocoma_msgs::GetActiveController::Response &res);

  void notifyEmergencyStop(rocoma::ControllerManager::EmergencyStopType type);

 private:
  //! Ros services
  ros::ServiceServer switchControllerService_;
  ros::ServiceServer emergencyStopService_;
  ros::ServiceServer getAvailableControllersService_;
  ros::ServiceServer getActiveControllerService_;

  //! Notify an emergency stop
  void publishEmergencyState(bool isOk);
  ros::Publisher emergencyStopStatePublisher_;
  any_msgs::State emergencyStopStateMsg_;

  //! Class loaders
  pluginlib::ClassLoader< rocoma_plugin::FailproofControllerPluginInterface<State_, Command_> > failproofControllerLoader_;
  pluginlib::ClassLoader< rocoma_plugin::EmergencyControllerPluginInterface<State_, Command_> > emergencyControllerLoader_;
  pluginlib::ClassLoader< rocoma_plugin::ControllerPluginInterface<State_, Command_> > controllerLoader_;

};
}

#include <rocoma_ros/ControllerManagerRos.tpp>
