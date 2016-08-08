#pragma once

// rocoma
#include "rocoma/ControllerManager.hpp"
#include "rocoma/controllers/ControllerAdapter.hpp"

// rocoma plugin
#include "rocoma_plugin/interfaces/ControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/EmergencyControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/FailproofControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/ControllerRosPluginInterface.hpp"
#include "rocoma_plugin/interfaces/EmergencyControllerRosPluginInterface.hpp"

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

//! Extension of the Controller Manager to ROS
/*! Functionalities of the rocoma controller manager are wrapped as ros services.
 *  Controllers can be loaded using the ros pluginlib.
 *
 */
template<typename State_, typename Command_>
class ControllerManagerRos : public rocoma::ControllerManager {

 public:

  //! Struct to simplify the adding of a controller pair
  struct ControllerPairOptions {

    /*! Constructor
     * @param controllerName            Name of the controller
     * @param emergencyControllerName   Name of the corresponding emergency controller
     * @param isRosController           True if the controller is of type ControllerRosPlugin
     * @param isRosEmergencyController  True if the emergency controller is of type EmergencyControllerRosPlugin
     * @returns object of type ControllerPairOptions
     */
    ControllerPairOptions(const std::string & controllerName,
                          const std::string & emergencyControllerName,
                          const bool & isRosController,
                          const bool & isRosEmergencyController):
                            controllerName_(controllerName),
                            emergencyControllerName_(emergencyControllerName),
                            isRosController_(isRosController),
                            isRosEmergencyController_(isRosEmergencyController)
    {

    }

    std::string controllerName_;
    std::string emergencyControllerName_;
    bool isRosController_;
    bool isRosEmergencyController_;
  };

 public:

  /*! Constructor
   * @param nodeHandle              Ros nodehandle (register services, set to ros controllers)
   * @param scopedStateName         Name of the robot state class, including namespaces  (e.g. "myStatePkg::State")
   * @param scopedCommandName       Name of the robot command class, including namespaces  (e.g. "myCommandPkg::Command")
   * @returns object of type ControllerManagerRos
   */
  ControllerManagerRos(ros::NodeHandle& nodeHandle,
                       const std::string & scopedStateName,
                       const std::string & scopedCommandName);

  //! Default destructor
  virtual ~ControllerManagerRos();


  /*! Add a single controller pair to the manager
   * @param options         options containing names and ros flags
   * @param state           robot state pointer
   * @param command         robot command pointer
   * @param mutexState      mutex protecting robot state pointer
   * @param mutexCommand    mutex protecting robot command pointer
   * @returns true iff added controller pair successfully
   */
  bool setupControllerPair(const ControllerPairOptions & options,
                           std::shared_ptr<State_> state,
                           std::shared_ptr<Command_> command,
                           std::shared_ptr<boost::shared_mutex> mutexState,
                           std::shared_ptr<boost::shared_mutex> mutexCommand);

  /*! Add the failproof controller to the controller manager
   * @param controllerPluginName  name of the failproof controller
   * @param state                 robot state pointer
   * @param command               robot command pointer
   * @param mutexState            mutex protecting robot state pointer
   * @param mutexCommand          mutex protecting robot command pointer
   * @returns true iff the failproof controller was added successfully
   */
  bool setupFailproofController(const std::string & controllerPluginName,
                                std::shared_ptr<State_> state,
                                std::shared_ptr<Command_> command,
                                std::shared_ptr<boost::shared_mutex> mutexState,
                                std::shared_ptr<boost::shared_mutex> mutexCommand);

  /*! Add a vector of controller pairs and a failproof controller to the manager
   * @param failproofControllerName name of the failproof controller
   * @param controllerOptions       options containing names and ros flags of all controller pairs
   * @param state                   robot state pointer
   * @param command                 robot command pointer
   * @param mutexState              mutex protecting robot state pointer
   * @param mutexCommand            mutex protecting robot command pointer
   * @returns true iff all controllers were added successfully
   */
  bool setupControllers(const std::string & failproofControllerName,
                        const std::vector<ControllerPairOptions> & controllerOptions,
                        std::shared_ptr<State_> state,
                        std::shared_ptr<Command_> command,
                        std::shared_ptr<boost::shared_mutex> mutexState,
                        std::shared_ptr<boost::shared_mutex> mutexCommand);

  /*! Emergency stop service callback, triggers an emergency stop of the current controller
   * @param req   empty request
   * @param res   empty response
   * @return true iff successful
   */
  bool emergencyStop(rocoma_msgs::EmergencyStop::Request  &req,
                     rocoma_msgs::EmergencyStop::Response &res);

  /*! Switch controller service callback, switches to a new controller
   * @param req   contains name of the new controller
   * @param res   contains the result of the switching
   * @return true iff successful
   */
  bool switchController(rocoma_msgs::SwitchController::Request  &req,
                        rocoma_msgs::SwitchController::Response &res);

  /*! Get available controllers service callback, returns a list of all available controllers
   * @param req   empty request
   * @param res   contains vector of strings with the available controller names
   * @return true iff successful
   */
  bool getAvailableControllers(rocoma_msgs::GetAvailableControllers::Request &req,
                               rocoma_msgs::GetAvailableControllers::Response &res);

  /*! Get active controller service callback, returns the name of the currently active controller
   * @param req   empty request
   * @param res   contains name of the currently active controller
   * @return true iff successful
   */
  bool getActiveController(rocoma_msgs::GetActiveController::Request &req,
                           rocoma_msgs::GetActiveController::Response &res);

  /*! Inform other nodes (via message) when an emergency stop was triggered
   * @param type   type of the emergency stop
   */
  void notifyEmergencyStop(rocoma::ControllerManager::EmergencyStopType type);

 private:
  /*! Publish on the emergency stop topic
   * @param isOk   publish isOk on topic
   */
  void publishEmergencyState(bool isOk);

 private:
  //! Ros node handle
  ros::NodeHandle nodeHandle_;

  //! Switch controller service
  ros::ServiceServer switchControllerService_;
  //! Emergency stop service
  ros::ServiceServer emergencyStopService_;
  //! Get available controllers service
  ros::ServiceServer getAvailableControllersService_;
  //! Get active controller service
  ros::ServiceServer getActiveControllerService_;

  //! Emergency stop publisher
  ros::Publisher emergencyStopStatePublisher_;
  //! Emergency stop message
  any_msgs::State emergencyStopStateMsg_;

  //! Failproof controller class loader
  pluginlib::ClassLoader< rocoma_plugin::FailproofControllerPluginInterface<State_, Command_> > failproofControllerLoader_;
  //! Emergency controller class loader
  pluginlib::ClassLoader< rocoma_plugin::EmergencyControllerPluginInterface<State_, Command_> > emergencyControllerLoader_;
  //! Emergency controller ROS class loader
  pluginlib::ClassLoader< rocoma_plugin::EmergencyControllerRosPluginInterface<State_, Command_> > emergencyControllerRosLoader_;
  //! Controller class loader
  pluginlib::ClassLoader< rocoma_plugin::ControllerPluginInterface<State_, Command_> > controllerLoader_;
  //! Controller ROS class loader
  pluginlib::ClassLoader< rocoma_plugin::ControllerRosPluginInterface<State_, Command_> > controllerRosLoader_;

};

}

#include <rocoma_ros/ControllerManagerRos.tpp>
