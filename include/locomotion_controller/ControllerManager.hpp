/*
 * ControllerManager.hpp
 *
 *  Created on: Nov 10, 2014
 *      Author: gech
 */

#ifndef CONTROLLERMANAGER_HPP_
#define CONTROLLERMANAGER_HPP_

#include <ros/ros.h>
#include <locomotion_controller/SwitchController.h>

#include <locomotion_controller/Model.hpp>



#include "TaskRobotBase.hpp"
#include "NoTask_Task.hpp"


#include <boost/ptr_container/ptr_vector.hpp>

namespace locomotion_controller {

class ControllerManager;
void add_locomotion_controllers(locomotion_controller::ControllerManager* manager, model::Model* model);

class ControllerManager
{
 public:
  typedef robotTask::TaskRobotBase* ControllerPtr;
 public:
  ControllerManager();
  virtual ~ControllerManager();

  void updateController();
  void setupControllers(double dt, double time, model::Model* model);
  void addController(ControllerPtr controller);
  bool switchController(locomotion_controller::SwitchController::Request  &req,
                                 locomotion_controller::SwitchController::Response &res);
 protected:
  double time_;
  double timeStep_;
  bool isInitializingTask_;
  boost::ptr_vector<robotTask::TaskRobotBase> controllers_;
  robotTask::TaskRobotBase* activeController_;
};

} /* namespace locomotion_controller */

#endif /* CONTROLLERMANAGER_HPP_ */
