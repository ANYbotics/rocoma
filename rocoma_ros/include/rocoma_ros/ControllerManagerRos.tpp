#include <rocoma_ros/ControllerManagerRos.hpp>

namespace rocoma_ros {

template<typename State_, typename Command_>
ControllerManagerRos<State_,Command_>::ControllerManagerRos(ros::NodeHandle& nodeHandle, const std::string & scopedStateName,
                                                            const std::string & scopedCommandName):
                                                            rocoma::ControllerManager(),
                                                            nodeHandle_(nodeHandle),
                                                            failproofControllerLoader_("rocoma_plugin", "rocoma_plugin::FailproofControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                            emergencyControllerLoader_("rocoma_plugin", "rocoma_plugin::EmergencyControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                            emergencyControllerRosLoader_("rocoma_plugin", "rocoma_plugin::EmergencyControllerRosPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                            controllerLoader_("rocoma_plugin", "rocoma_plugin::ControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                            controllerRosLoader_("rocoma_plugin", "rocoma_plugin::ControllerRosPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">")

{
  // initialize services
  switchControllerService_ = nodeHandle.advertiseService("switch_controller", &ControllerManagerRos::switchController, this);
  getAvailableControllersService_ = nodeHandle.advertiseService("get_available_controllers", &ControllerManagerRos::getAvailableControllers, this);
  getActiveControllerService_ = nodeHandle.advertiseService("get_active_controller", &ControllerManagerRos::getActiveController, this);
  emergencyStopService_ = nodeHandle.advertiseService("emergency_stop", &ControllerManagerRos::emergencyStop, this);

  // initialize publishers
  emergencyStopStatePublisher_.shutdown();
  emergencyStopStatePublisher_ = nodeHandle.advertise<any_msgs::State>("notify_emergency_stop", 1, true);
  publishEmergencyState(true);
}

template<typename State_, typename Command_>
ControllerManagerRos<State_,Command_>::~ControllerManagerRos() {

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupControllerPair(const ControllerPairOptions & options,
                                                                std::shared_ptr<State_> state,
                                                                std::shared_ptr<Command_> command,
                                                                std::shared_ptr<boost::shared_mutex> mutexState,
                                                                std::shared_ptr<boost::shared_mutex> mutexCommand) {

  //--- Add controller
  rocoma_plugin::ControllerPluginInterface<State_, Command_> * controller;

  try
  {
    if(options.isRosController_) {
      // Instantiate controller
      rocoma_plugin::ControllerRosPluginInterface<State_, Command_> * rosController = controllerRosLoader_.createUnmanagedInstance(options.controllerName_);
      // Set node handle
      rosController->setNodeHandle(nodeHandle_);
      controller = rosController;
    }
    else {
      controller = controllerLoader_.createUnmanagedInstance(options.controllerName_);
    }

    // Set state and command
    controller->setStateAndCommand(state, mutexState, command, mutexCommand);

  }
  catch(pluginlib::PluginlibException& ex)
  {
    //handle the class failing to load
    MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    MELO_WARN_STREAM("Could not setup controller: " << options.controllerName_ << "!");

    return false;
  }


  //--- Add emergency controller
  rocoma_plugin::EmergencyControllerPluginInterface<State_, Command_> * emgcyController = nullptr;

  if(!options.emergencyControllerName_.empty())
  {
    try
    {
      if(options.isRosEmergencyController_) {
        // Instantiate controller
        rocoma_plugin::EmergencyControllerRosPluginInterface<State_, Command_> * rosEmergencyController =
            emergencyControllerRosLoader_.createUnmanagedInstance(options.emergencyControllerName_);
        // Set node handle
        rosEmergencyController->setNodeHandle(nodeHandle_);
        emgcyController = rosEmergencyController;
      }
      else {
        emgcyController = emergencyControllerLoader_.createUnmanagedInstance(options.emergencyControllerName_);
      }

      // Set state and command
      emgcyController->setStateAndCommand(state, mutexState, command, mutexCommand);

    }
    catch(pluginlib::PluginlibException& ex)
    {
      //handle the class failing to load
      MELO_WARN("The plugin failed to load for some reason. Error: %s", ex.what());
      MELO_WARN_STREAM("Could not setup emergency controller: " << options.emergencyControllerName_ << "! Using failproof controller instead");
      emgcyController = nullptr;
    }
  } // endif

  // Set name to failproof if nullptr
  std::string emergencyControllerName = (emgcyController == nullptr)? "FailproofController" : options.emergencyControllerName_;

  // Add controller to the manager
  if(!this->addControllerPair(std::unique_ptr< roco::ControllerAdapterInterface >(controller),
                              std::unique_ptr< roco::EmergencyControllerAdapterInterface >(emgcyController)))
  {
    MELO_WARN_STREAM("Could not add controller pair ( " << options.controllerName_ << " / "
                     << emergencyControllerName << " ) to controller manager!");
    return false;
  }

  // Inform user
  MELO_INFO_STREAM("Successfully added controller pair ( " << options.controllerName_ << " / "
                   << emergencyControllerName << " ) to controller manager!");

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupFailproofController(const std::string & controllerPluginName,
                                                                     std::shared_ptr<State_> state,
                                                                     std::shared_ptr<Command_> command,
                                                                     std::shared_ptr<boost::shared_mutex> mutexState,
                                                                     std::shared_ptr<boost::shared_mutex> mutexCommand) {
  try
  {
    // Instantiate controller
    rocoma_plugin::FailproofControllerPluginInterface<State_, Command_> * controller =
        failproofControllerLoader_.createUnmanagedInstance(controllerPluginName);

    // Set state and command
    controller->setStateAndCommand(state, mutexState, command, mutexCommand);

    // Add controller to the manager
    if(!this->setFailproofController(std::unique_ptr< rocoma_plugin::FailproofControllerPluginInterface<State_,Command_> >(controller))) {
      MELO_WARN_STREAM("Could not add failproof controller: " << controllerPluginName << " to controller manager!");
      return false;
    }

    // Inform user
    MELO_INFO_STREAM("Succesfully setup failproof controller: " << controllerPluginName << "!");

  }
  catch(pluginlib::PluginlibException& ex)
  {
    //handle the class failing to load
    MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    MELO_WARN_STREAM("Could not setup failproof controller: " << controllerPluginName << "!");
    return false;
  }

  return true;

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupControllers(const std::string & failproofControllerName,
                                                             const std::vector< ControllerPairOptions > & controllerNameMap,
                                                             std::shared_ptr<State_> state,
                                                             std::shared_ptr<Command_> command,
                                                             std::shared_ptr<boost::shared_mutex> mutexState,
                                                             std::shared_ptr<boost::shared_mutex> mutexCommand) {
  // add failproof controller to manager
  bool success = setupFailproofController(failproofControllerName, state, command, mutexState, mutexCommand);

  // if failproof controller can not be added abort immediately
  if( !success )
  {
    MELO_FATAL("Failproof controller could not be added! ABORT!");
    exit(-1);
  }

  // add emergency controllers to manager
  for(auto& controllerPair : controllerNameMap)
  {
    success = success && setupControllerPair(controllerPair, state, command, mutexState, mutexCommand);
  }

  return success;

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::emergencyStop(rocoma_msgs::EmergencyStop::Request& req,
                                                          rocoma_msgs::EmergencyStop::Response& res) {
  return rocoma::ControllerManager::emergencyStop();
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::switchController(rocoma_msgs::SwitchController::Request& req,
                                                             rocoma_msgs::SwitchController::Response& res) {

  switch(rocoma::ControllerManager::switchController(req.name)) {
    case rocoma::ControllerManager::SwitchResponse::ERROR:
      res.status = res.STATUS_ERROR;
      break;
    case rocoma::ControllerManager::SwitchResponse::NOTFOUND:
      res.status = res.STATUS_NOTFOUND;
      break;
    case rocoma::ControllerManager::SwitchResponse::RUNNING:
      res.status = res.STATUS_RUNNING;
      break;
    case rocoma::ControllerManager::SwitchResponse::SWITCHING:
      res.status = res.STATUS_SWITCHED;
      break;
  }

  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::getAvailableControllers( rocoma_msgs::GetAvailableControllers::Request& req,
                                                                     rocoma_msgs::GetAvailableControllers::Response& res) {
  res.available_controllers = this->getAvailableControllerNames();
  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::getActiveController(rocoma_msgs::GetActiveController::Request& req,
                                                                rocoma_msgs::GetActiveController::Response& res) {
  res.active_controller = this->getActiveControllerName();
  return true;
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::notifyEmergencyStop(rocoma::ControllerManager::EmergencyStopType type) {
  // TODO react differently depending on the emergency stop type
  publishEmergencyState(false);
  publishEmergencyState(true);
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::publishEmergencyState(bool isOk) {
  // Fill msg
  emergencyStopStateMsg_.stamp = ros::Time::now();
  emergencyStopStateMsg_.is_ok = isOk;

  // Publish message
  any_msgs::StatePtr stateMsg( new any_msgs::State(emergencyStopStateMsg_) );
  emergencyStopStatePublisher_.publish( any_msgs::StateConstPtr(stateMsg) );
}

}
