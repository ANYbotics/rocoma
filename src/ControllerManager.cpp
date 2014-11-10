/*
 * ControllerManager.cpp
 *
 *  Created on: Nov 10, 2014
 *      Author: gech
 */


#include "locomotion_controller/ControllerManager.hpp"


#ifdef USE_TASK_LOCODEMO
#include "LocoDemo_Task.hpp"
#endif

namespace locomotion_controller {

ControllerManager::ControllerManager() :
    isInitializingTask_(false),
    controllers_(),
    activeController_(nullptr)
{

}

ControllerManager::~ControllerManager()
{
}

void ControllerManager::setupControllers(double dt, double time, model::Model* model)  {



/* Create no task, which is active until estimator converged*/
  controllers_.push_back(new robotTask::NoTask(model->getRobotModel()));
  robotTask::TaskRobotBase* controller = &controllers_.back();
  activeController_ = controller;
  controller->setTime(time);
  controller->setTimeStep(dt);
  if (!controller->add()) {
    throw std::runtime_error("Could not add 'no task'!");
  }



#ifdef USE_TASK_LOCODEMO
  controllers_.push_back(new robotTask::LocoDemo(model->getRobotModel(), model->getTerrainModel()));
  controller = &controllers_.back();
  controller->setTime(time);
  controller->setTimeStep(dt);
  ROS_INFO("Added Task %s.", controller->getName().c_str());
  if (!controller->add()) {
    throw std::runtime_error("Could not add the task!");
  }
#endif

}


void ControllerManager::updateController(double dt, double time) {

  if (isInitializingTask_) {

    /* initialize the task */
    if (!activeController_->initTask()) {
      throw std::runtime_error("Could not initialize the task!");
    }
    isInitializingTask_ = false;
    ROS_INFO("Initialized controller %s", activeController_->getName().c_str());
  }

  activeController_->setTime(time);
  activeController_->runTask();
//  std::cout << "updateController() Qb:\n" << model_.getRobotModel()->q().getQb() << std::endl;

}


bool ControllerManager::switchController(locomotion_controller::SwitchController::Request  &req,
                               locomotion_controller::SwitchController::Response &res)
{
  std::string reqTaskName = req.name;
  if (req.name == "EmergencyStop") {
    ROS_INFO("Emergency Stop!");
    reqTaskName = "No Task";
  }

  //--- Check if controller is already active
  if (reqTaskName == activeController_->getName()) {
    res.status = res.STATUS_RUNNING;
    ROS_INFO("Controller is already running!");
    return true;
  }

  for (auto& controller : controllers_) {
    if (reqTaskName == controller.getName()) {
      activeController_ = &controller;
      res.status = res.STATUS_SWITCHED;
      isInitializingTask_ = true;
      ROS_INFO("Switched to controller %s", activeController_->getName().c_str());
      return true;
    }
  }
  res.status = res.STATUS_NOTFOUND;
  ROS_INFO("Controller %s not found!", reqTaskName.c_str());
  return true;
}

} /* namespace locomotion_controller */
