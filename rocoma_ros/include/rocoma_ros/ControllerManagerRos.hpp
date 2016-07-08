#pragma once

#include <string>
#include <memory>

#include <pluginlib/class_loader.h>
#include <rocoma/ControllerManager.hpp>

namespace rocoma_ros {

  class ControllerManagerRos {

    ControllerManagerRos();
    ~ControllerManagerRos();


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
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        return false;
      }
    }


   private:
    rocoma::ControllerManager controllerManager_;
  };
}
