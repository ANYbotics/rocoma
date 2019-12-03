/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Complex tests to test combinations.
 */

#include <gtest/gtest.h>

#include "include/TestControllerManager.hpp"

namespace rocoma {

TEST_F(TestControllerManager, doesNotAllowMultipleSwitches) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  startSwitchController(sleepyControllerA_);
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, controllerManager_.switchController(simpleControllerB_));
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::SWITCHING, cancelSwitchController());
}

TEST_F(TestControllerManager, doesNotAllowSwitchingOnEstop) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  startSwitchController(sleepyControllerA_);
  emergencyStop();
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, cancelSwitchController());
}

TEST_F(TestControllerManager, allowsSecondEstopsWhileStopping) {  // NOLINT
  clearEstopAndSwitchController(sleepyControllerA_);
  startEmergencyStop();
  emergencyStop();
  ASSERT_TRUE(isEmergencyStopRunning());  // Estop should still be runnning sleepy controller takes long to shutdown
  cancelEmergencyStop();
}

TEST_F(TestControllerManager, doesNotCallPrestopTwice) {  // NOLINT
  clearEstopAndSwitchController(preStopCheckControllerA_);
  startSwitchController(sleepyControllerA_);
  emergencyStop();
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, cancelSwitchController());
  ASSERT_EQ(command_->getValue(), 1.0);
}

TEST_F(TestControllerManager, allowsSwitchingAfterEstopDuringSwitch) {  // NOLINT
  runControllerManagerUpdateFor(0.025);
  clearEstopAndSwitchController(simpleControllerA_);
  startSwitchController(sleepyControllerA_);
  emergencyStop();
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, cancelSwitchController());
  clearEstopAndSwitchController(simpleControllerA_);
  switchController(sleepyControllerA_);
  switchController(simpleControllerA_);
  cancelControllerManagerUpdate();
}

}  // namespace rocoma
