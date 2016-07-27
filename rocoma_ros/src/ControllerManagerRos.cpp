#include <rocoma_ros/ControllerManagerRos.hpp>

namespace rocoma_ros {

ControllerManagerRos::ControllerManagerRos(ros::NodeHandle& nodeHandle):
    controllerManager_()
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

ControllerManagerRos::~ControllerManagerRos()
{

}

bool ControllerManagerRos::emergencyStop(rocoma_msgs::EmergencyStop::Request& req,
                                         rocoma_msgs::EmergencyStop::Response& res)
{
  return controllerManager_.emergencyStop();
}

bool ControllerManagerRos::switchController(rocoma_msgs::SwitchController::Request& req,
                                            rocoma_msgs::SwitchController::Response& res)
{

  switch(controllerManager_.switchController(req.name)) {
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

bool ControllerManagerRos::getAvailableControllers(
    rocoma_msgs::GetAvailableControllers::Request& req,
    rocoma_msgs::GetAvailableControllers::Response& res)
{
  res.available_controllers = controllerManager_.getAvailableControllerNames();
  return true;
}

bool ControllerManagerRos::getActiveController(rocoma_msgs::GetActiveController::Request& req,
                                               rocoma_msgs::GetActiveController::Response& res)
{
  res.active_controller = controllerManager_.getActiveControllerName();
  return true;
}

void ControllerManagerRos::reactOnEmergencyStop(rocoma::EmergencyStopObserver::EmergencyStopType type) {
  // TODO react differently depending on the emergency stop type
  publishEmergencyState(false);
  publishEmergencyState(true);
}

void ControllerManagerRos::publishEmergencyState(bool isOk) {
  emergencyStopStateMsg_.stamp = ros::Time::now();
  emergencyStopStateMsg_.is_ok = isOk;

  any_msgs::StatePtr stateMsg( new any_msgs::State(emergencyStopStateMsg_) );
  emergencyStopStatePublisher_.publish( any_msgs::StateConstPtr(stateMsg) );
}

}
