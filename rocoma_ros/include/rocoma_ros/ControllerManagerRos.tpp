#include <rocoma_ros/ControllerManagerRos.hpp>

namespace rocoma_ros {

template<typename State_, typename Command_>
ControllerManagerRos<State_,Command_>::ControllerManagerRos(ros::NodeHandle& nodeHandle, const std::string & scopedStateName,
                                                            const std::string & scopedCommandName):
                                                            rocoma::ControllerManager(),
                                                            failproofControllerLoader_("rocoma_plugin", "rocoma_plugin::FailproofControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                            emergencyControllerLoader_("rocoma_plugin", "rocoma_plugin::EmergencyControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                            controllerLoader_("rocoma_plugin", "rocoma_plugin::ControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">")
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
ControllerManagerRos<State_,Command_>::~ControllerManagerRos()
{

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupController(const std::string & controllerPluginName,
                                                            std::shared_ptr<State_> state,
                                                            std::shared_ptr<Command_> command,
                                                            std::shared_ptr<boost::shared_mutex> mutexState,
                                                            std::shared_ptr<boost::shared_mutex> mutexCommand,
                                                            std::shared_ptr<any_worker::WorkerManager> workerManager) {

  rocoma_plugin::ControllerPluginInterface<State_, Command_> * controller;

  try
  {
    // Instantiate controller
    controller = controllerLoader_.createUnmanagedInstance(controllerPluginName);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    //handle the class failing to load
    MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    MELO_WARN_STREAM("Could not setup controller: " << controllerPluginName << "!");

    return false;
  }

  // Check for null
  if(controller == nullptr) {
    MELO_WARN_STREAM("Could not setup controller: " << controllerPluginName << "! (nullptr)");
    return false;
  }

  // Set state and command
  controller->setStateAndCommand(state, mutexState, command, mutexCommand);

  // Add controller to the manager
  if( !this->addController(std::unique_ptr< rocoma_plugin::ControllerPluginInterface<State_,Command_> >(controller)) ) {
    MELO_WARN_STREAM("Could not add controller: " << controllerPluginName << " to controller manager!");
    return false;
  }

  // Inform user
  MELO_INFO_STREAM("Succesfully setup controller: " << controllerPluginName << "!");

  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupEmergencyController(const std::string & controllerPluginName,
                                                                     std::shared_ptr<State_> state,
                                                                     std::shared_ptr<Command_> command,
                                                                     std::shared_ptr<boost::shared_mutex> mutexState,
                                                                     std::shared_ptr<boost::shared_mutex> mutexCommand,
                                                                     std::shared_ptr<any_worker::WorkerManager> workerManager) {

  rocoma_plugin::EmergencyControllerPluginInterface<State_, Command_> * controller;
  try
  {
    // Instantiate controller
    controller = emergencyControllerLoader_.createUnmanagedInstance(controllerPluginName);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    //handle the class failing to load
    MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    MELO_WARN_STREAM("Could not setup emergency controller: " << controllerPluginName << "!");
    return false;
  }

  // Check for null
  if(controller == nullptr) {
    MELO_WARN_STREAM("Could not setup emergency controller: " << controllerPluginName << "! (nullptr)");
    return false;
  }

  // Set state and command
  controller->setStateAndCommand(state, mutexState, command, mutexCommand);

  // Add controller to the manager
  if(!this->addEmergencyController(std::unique_ptr< rocoma_plugin::EmergencyControllerPluginInterface<State_,Command_> >(controller))) {
    MELO_WARN_STREAM("Could not add emergency controller: " << controllerPluginName << " to controller manager!");
    return false;
  }

  // Inform user
  MELO_INFO_STREAM("Succesfully setup emergency controller: " << controllerPluginName << "!");

  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupFailproofController(const std::string & controllerPluginName,
                                                                     std::shared_ptr<State_> state,
                                                                     std::shared_ptr<Command_> command,
                                                                     std::shared_ptr<boost::shared_mutex> mutexState,
                                                                     std::shared_ptr<boost::shared_mutex> mutexCommand,
                                                                     std::shared_ptr<any_worker::WorkerManager> workerManager) {

  rocoma_plugin::FailproofControllerPluginInterface<State_, Command_> * controller;
  try
  {
    // Instantiate controller
    controller = failproofControllerLoader_.createUnmanagedInstance(controllerPluginName);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    //handle the class failing to load
    MELO_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    MELO_WARN_STREAM("Could not setup failproof controller: " << controllerPluginName << "!");
    return false;
  }

  // Check for null
  if(controller == nullptr) {
    MELO_WARN_STREAM("Could not setup failproof controller: " << controllerPluginName << "! (nullptr)");
    return false;
  }

  // Set state and command
  controller->setStateAndCommand(state, mutexState, command, mutexCommand);

  // Add controller to the manager
  if(!this->setFailproofController(std::unique_ptr< rocoma_plugin::FailproofControllerPluginInterface<State_,Command_> >(controller))) {
    MELO_WARN_STREAM("Could not add failproof controller: " << controllerPluginName << " to controller manager!");
    return false;
  }

  // Inform user
  MELO_INFO_STREAM("Succesfully setup failproof controller: " << controllerPluginName << "!");

  return true;

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupControllers(const std::string & failproofControllerName,
                                                             const std::vector<std::string> & emergencyControllerNames,
                                                             const std::vector<std::string> & controllerNames,
                                                             std::shared_ptr<State_> state,
                                                             std::shared_ptr<Command_> command,
                                                             std::shared_ptr<boost::shared_mutex> mutexState,
                                                             std::shared_ptr<boost::shared_mutex> mutexCommand,
                                                             std::shared_ptr<any_worker::WorkerManager> workerManager) {
  // add failproof controller to manager
  if( !setupFailproofController(failproofControllerName, state, command, mutexState, mutexCommand, workerManager) )
  {
    MELO_FATAL("Failproof controller could not be added! ABORT!");
    exit(-1);
  }

  // add emergency controllers to manager
  for(auto controller : emergencyControllerNames)
  {
    setupEmergencyController(controller, state, command, mutexState, mutexCommand, workerManager);
  }

  // setup normal controllers
  for(auto controller : controllerNames)
  {
    setupController(controller, state, command, mutexState, mutexCommand, workerManager);
  }

}



template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::emergencyStop(rocoma_msgs::EmergencyStop::Request& req,
                                                          rocoma_msgs::EmergencyStop::Response& res)
                                                          {
  return rocoma::ControllerManager::emergencyStop();
                                                          }

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::switchController(rocoma_msgs::SwitchController::Request& req,
                                                             rocoma_msgs::SwitchController::Response& res)
                                                             {

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
    case rocoma::ControllerManager::SwitchResponse::SWITCHED:
      res.status = res.STATUS_SWITCHED;
      break;
  }

  return true;
                                                             }

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::getAvailableControllers(
    rocoma_msgs::GetAvailableControllers::Request& req,
    rocoma_msgs::GetAvailableControllers::Response& res)
    {
  res.available_controllers = this->getAvailableControllerNames();
  return true;
    }

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::getActiveController(rocoma_msgs::GetActiveController::Request& req,
                                                                rocoma_msgs::GetActiveController::Response& res)
                                                                {
  res.active_controller = this->getActiveControllerName();
  return true;
                                                                }

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::reactOnEmergencyStop(rocoma::EmergencyStopObserver::EmergencyStopType type) {
  // TODO react differently depending on the emergency stop type
  publishEmergencyState(false);
  publishEmergencyState(true);
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::publishEmergencyState(bool isOk) {
  emergencyStopStateMsg_.stamp = ros::Time::now();
  emergencyStopStateMsg_.is_ok = isOk;

  any_msgs::StatePtr stateMsg( new any_msgs::State(emergencyStopStateMsg_) );
  emergencyStopStatePublisher_.publish( any_msgs::StateConstPtr(stateMsg) );
}

}
