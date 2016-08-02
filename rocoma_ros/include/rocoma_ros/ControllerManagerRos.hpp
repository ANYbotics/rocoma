#pragma once

// rocoma
#include "rocoma/ControllerManager.hpp"
#include "rocoma/controllers/ControllerAdapter.hpp"
#include "rocoma/common/EmergencyStopObserver.hpp"

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
#include <memory>

namespace rocoma_ros {

class ControllerManagerRos : public rocoma::EmergencyStopObserver{

 public:
  ControllerManagerRos(ros::NodeHandle& nodeHandle);
  ~ControllerManagerRos();

  const rocoma::ControllerManager& getControllerManager()
  {
    return controllerManager_;
  }

  template<typename State_, typename Command_>
  bool addController(const std::string & controllerPluginName,
                     const std::string & scopedStateName,
                     const std::string & scopedCommandName,
                     double timeStep,
                     std::shared_ptr<State_> state,
                     std::shared_ptr<Command_> command,
                     std::shared_ptr<boost::shared_mutex> mutexState,
                     std::shared_ptr<boost::shared_mutex> mutexCommand,
                     std::shared_ptr<any_worker::WorkerManager> workerManager)
  {
    // TODO remove when done debugging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    // Convinience typedef
    using PluginBaseType = rocoma_plugin::ControllerPluginInterface<State_, Command_>;

    // Create plugin loader
    pluginlib::ClassLoader<PluginBaseType> controller_loader("rocoma_plugin",
          "rocoma_plugin::ControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">");

    try
    {
      // Instantiate controller
      boost::shared_ptr<PluginBaseType> controller = controller_loader.createInstance(controllerPluginName);

      // Initialize Controller
      controller->setStateAndCommand(state, mutexState, command, mutexCommand);

      // Transfer ownership of the controller to the controller manager
      // FIXME is the controller ptr still valid when it goes out of scope. Hopefully yes because of move
      MELO_INFO_STREAM("Adding controller " << controller->getControllerName() << " to the controller manager.");
      controllerManager_.addController(std::unique_ptr<roco::ControllerAdapterInterface>(controller.get()));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      //handle the class failing to load
      MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      return false;
    }
  }

  template<typename State_, typename Command_>
  bool addEmergencyController( const std::string & controllerPluginName,
                               const std::string & scopedStateName,
                               const std::string & scopedCommandName,
                               double timeStep,
                               std::shared_ptr<State_> state,
                               std::shared_ptr<Command_> command,
                               std::shared_ptr<boost::shared_mutex> mutexState,
                               std::shared_ptr<boost::shared_mutex> mutexCommand,
                               std::shared_ptr<any_worker::WorkerManager> workerManager)
  {
    // TODO remove when done debugging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    // Convinience typedef
    using PluginBaseType = rocoma_plugin::EmergencyControllerPluginInterface<State_, Command_>;

    // Create plugin loader
    pluginlib::ClassLoader<PluginBaseType> controller_loader("rocoma_plugin",
          "rocoma_plugin::EmergencyControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">");

    try
    {
      // Instantiate controller
      boost::shared_ptr<PluginBaseType> controller = controller_loader.createInstance(controllerPluginName);

      // Initialize Controller
      controller->setStateAndCommand(state, mutexState, command, mutexCommand);

      // Transfer ownership of the controller to the controller manager
      // FIXME is the controller ptr still valid when it goes out of scope. Hopefully yes because of move
      MELO_INFO_STREAM("Adding controller " << controller->getControllerName() << " to the controller manager.");
      controllerManager_.addController(std::unique_ptr<roco::ControllerAdapterInterface>(controller.get()));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      //handle the class failing to load
      MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      return false;
    }
  }

  template<typename State_, typename Command_>
  bool addFailproofController( const std::string & controllerPluginName,
                               const std::string & scopedStateName,
                               const std::string & scopedCommandName,
                               double timeStep,
                               std::shared_ptr<State_> state,
                               std::shared_ptr<Command_> command,
                               std::shared_ptr<boost::shared_mutex> mutexState,
                               std::shared_ptr<boost::shared_mutex> mutexCommand,
                               std::shared_ptr<any_worker::WorkerManager> workerManager)
  {
    // TODO remove when done debugging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    // Convinience typedef
    using PluginBaseType = rocoma_plugin::FailproofControllerPluginInterface<State_, Command_>;

    // Create plugin loader
    pluginlib::ClassLoader<PluginBaseType> controller_loader("rocoma_plugin",
          "rocoma_plugin::FailproofControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">");

    try
    {
      // Instantiate controller
      boost::shared_ptr<PluginBaseType> controller = controller_loader.createInstance(controllerPluginName);

      // Initialize Controller
      controller->setStateAndCommand(state, mutexState, command, mutexCommand);

      // Transfer ownership of the controller to the controller manager
      // FIXME is the controller ptr still valid when it goes out of scope. Hopefully yes because of move
      MELO_INFO_STREAM("Adding controller " << controller->getControllerName() << " to the controller manager.");
      //controllerManager_.addController(std::unique_ptr<roco::FailproofControllerAdapterInterface>(controller.get()));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      //handle the class failing to load
      MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      return false;
    }
  }

  bool emergencyStop(rocoma_msgs::EmergencyStop::Request  &req,
                     rocoma_msgs::EmergencyStop::Response &res);

  bool switchController(rocoma_msgs::SwitchController::Request  &req,
                        rocoma_msgs::SwitchController::Response &res);

  bool getAvailableControllers(rocoma_msgs::GetAvailableControllers::Request &req,
                               rocoma_msgs::GetAvailableControllers::Response &res);

  bool getActiveController(rocoma_msgs::GetActiveController::Request &req,
                           rocoma_msgs::GetActiveController::Response &res);

  void reactOnEmergencyStop(rocoma::EmergencyStopObserver::EmergencyStopType type);

 private:
  //! Controller Manager
  rocoma::ControllerManager controllerManager_;

  //! Ros services
  ros::ServiceServer switchControllerService_;
  ros::ServiceServer emergencyStopService_;
  ros::ServiceServer getAvailableControllersService_;
  ros::ServiceServer getActiveControllerService_;

  //! Notify an emergency stop
  void publishEmergencyState(bool isOk);
  ros::Publisher emergencyStopStatePublisher_;
  any_msgs::State emergencyStopStateMsg_;

};
}
