#pragma once

// rocoma
#include "rocoma/ControllerManager.hpp"
#include "rocoma/controllers/ControllerAdapter.hpp"
#include "rocoma/common/EmergencyStopObserver.hpp"
#include "rocoma/plugin/ControllerPluginInterface.hpp"

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
  bool addController(const std::string & packageName,
                     const std::string & namespaceName,
                     const std::string & className,
                     double dt,
                     std::shared_ptr<State_> state,
                     std::shared_ptr<Command_> command,
                     std::shared_ptr<boost::shared_mutex> mutexState,
                     std::shared_ptr<boost::shared_mutex> mutexCommand,
                     std::shared_ptr<any_worker::WorkerManager> workerManager)
  {
if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}
    using BaseType = rocoma::ControllerPluginInterface<State_, Command_>;

    const std::string scopedClassName = namespaceName + std::string("::") + className;
    pluginlib::ClassLoader<BaseType> controller_loader("rocoma", "rocoma::ControllerPluginInterface<rocomaex_model::State, rocomaex_model::Command>");

    try
    {
      boost::shared_ptr<BaseType> controller = controller_loader.createInstance("Controller1Adapter");
      controller->setStateAndCommand(state, mutexState, command, mutexCommand);
      std::string name = "MyFirstPluginController";
      controller->setName(name);
      std::cout<<"Add controller "<<controller->getControllerName()<<std::endl;
      controllerManager_.addController(std::unique_ptr<roco::ControllerAdapterInterface>(controller.get()));
      // FIXME is the controller ptr still valid when it goes out of scope. Hopefully yes because of move
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
