/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Basic tests to assure trivial functionality.
 */

#include <gtest/gtest.h>

#include "include/TestControllerManager.hpp"

namespace rocoma {

TEST_F(TestControllerManager, startsInFailproof) {  // NOLINT
  checkActiveController(simpleFailProofController_);
}  // namespace rocoma

TEST_F(TestControllerManager, updatesSuccessfully) {  // NOLINT
  ASSERT_TRUE(controllerManager_.updateController());
}

TEST_F(TestControllerManager, startsWithUnclearedEstop) {  // NOLINT
  ASSERT_FALSE(controllerManager_.hasClearedEmergencyStop());
}

TEST_F(TestControllerManager, doesNotAllowSwitchesWithUnclearedEstop) {  // NOLINT
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, controllerManager_.switchController(simpleControllerA_));
}

TEST_F(TestControllerManager, canSwitchFromFailproofToControllerA) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
}

TEST_F(TestControllerManager, canSwitchFromControllerAToControllerB) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  switchController(simpleControllerB_);
}

TEST_F(TestControllerManager, doesNotLimitValidCommand) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  double okValue = RocoCommand::maxValue_ - 1.0;
  command_->setValue(okValue);
  ASSERT_TRUE(controllerManager_.updateController());
  ASSERT_EQ(okValue, command_->getValue());
}

TEST_F(TestControllerManager, doesLimitInvalidCommand) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  double failValue = RocoCommand::maxValue_ + 1.0;
  double limitedValue = RocoCommand::maxValue_;
  command_->setValue(failValue);
  ASSERT_TRUE(controllerManager_.updateController());
  ASSERT_EQ(limitedValue, command_->getValue());
}

TEST_F(TestControllerManager, staysInFailproofOnEstop) {  // NOLINT
  emergencyStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, staysInFailproofOnFailproofStop) {  // NOLINT
  failproofStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerAToFailproofOnEstop) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  emergencyStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerAToFailproofOnFailproofStop) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  failproofStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerBToEmergencyOnEstop) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerB_);
  emergencyStop();
  checkActiveController(simpleEmergencyController_);
  emergencyStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerBToFailproofOnFailproofStop) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerB_);
  failproofStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, cannotSwitchFromEmergencyControllerToControllerAWithUnclearedEstop) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  emergencyStop();
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, controllerManager_.switchController(simpleControllerB_));
}

TEST_F(TestControllerManager, canSwitchFromEmergencyControllerToControllerA) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  emergencyStop();
  clearEstopAndSwitchController(simpleControllerB_);
}

TEST_F(TestControllerManager, doesNotSwitchIfControllerAlreadyRunning) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::RUNNING, controllerManager_.switchController(simpleControllerA_));
  ASSERT_EQ(simpleControllerA_, controllerManager_.getActiveControllerName());
}

TEST_F(TestControllerManager, updatesSuccessfullyFor10ms) {  // NOLINT
  clearEstopAndSwitchController(simpleControllerA_);
  runControllerManagerUpdateFor(0.01);
  cancelControllerManagerUpdate();
}

}  // namespace rocoma
