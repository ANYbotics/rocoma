#pragma once

// rocoma
#include "rocoma/ControllerManager.hpp"
#include "rocoma/common/EmergencyStopObserver.hpp"

// rocoma msgs
#include "rocoma_msgs/GetAvailableControllers.h"
#include "rocoma_msgs/EmergencyStop.h"
#include "rocoma_msgs/SwitchController.h"
#include "rocoma_msgs/GetActiveController.h"

// any msgs
#include "any_msgs/State.h"

// message logger
#include "message_logger/message_logger.hpp"

// ros
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

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

    template<typename Controller_>
    bool addController(const std::string & packageName, const std::string & namespaceName, const std::string & className)
    {
      const std::string scopedClassName = namespaceName + std::string("::") + className;
      pluginlib::ClassLoader<Controller_> controller_loader(packageName, scopedClassName);

      try
      {
        boost::shared_ptr<Controller_> controller = controller_loader.createInstance(scopedClassName);
        controllerManager_.addController(std::unique_ptr<Controller_>(controller.get()));
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
