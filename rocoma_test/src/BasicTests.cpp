/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Basic tests to assure trivial functionality.
 */

#include <gtest/gtest.h>

#include "rocoma_test/TestControllerManager.hpp"

namespace rocoma_test {

TEST_F(TestControllerManager, startsInFailproof) { checkActiveController(simpleFailProofController_); }

TEST_F(TestControllerManager, updatesSuccessfully) { ASSERT_TRUE(controllerManager_.updateController()); }

TEST_F(TestControllerManager, startsWithUnclearedEstop) { ASSERT_FALSE(controllerManager_.hasClearedEmergencyStop()); }

TEST_F(TestControllerManager, doesNotAllowSwitchesWithUnclearedEstop) {
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, controllerManager_.switchController(simpleControllerA_));
}

TEST_F(TestControllerManager, canSwitchFromFailproofToControllerA) { clearEstopAndSwitchController(simpleControllerA_); }

TEST_F(TestControllerManager, canSwitchFromControllerAToControllerB) {
  clearEstopAndSwitchController(simpleControllerA_);
  switchController(simpleControllerB_);
}

TEST_F(TestControllerManager, doesNotLimitValidCommand) {
  clearEstopAndSwitchController(simpleControllerA_);
  double okValue = rocoma_test::RocoCommand::maxValue_ - 1.0;
  command_->setValue(okValue);
  ASSERT_TRUE(controllerManager_.updateController());
  ASSERT_EQ(okValue, command_->getValue());
}

TEST_F(TestControllerManager, doesLimitInvalidCommand) {
  clearEstopAndSwitchController(simpleControllerA_);
  double failValue = rocoma_test::RocoCommand::maxValue_ + 1.0;
  double limitedValue = rocoma_test::RocoCommand::maxValue_;
  command_->setValue(failValue);
  ASSERT_TRUE(controllerManager_.updateController());
  ASSERT_EQ(limitedValue, command_->getValue());
}

TEST_F(TestControllerManager, staysInFailproofOnEstop) {
  emergencyStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, staysInFailproofOnFailproofStop) {
  failproofStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerAToFailproofOnEstop) {
  clearEstopAndSwitchController(simpleControllerA_);
  emergencyStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerAToFailproofOnFailproofStop) {
  clearEstopAndSwitchController(simpleControllerA_);
  failproofStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerBToEmergencyOnEstop) {
  clearEstopAndSwitchController(simpleControllerB_);
  emergencyStop();
  checkActiveController(simpleEmergencyController_);
  emergencyStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, switchesFromControllerBToFailproofOnFailproofStop) {
  clearEstopAndSwitchController(simpleControllerB_);
  failproofStop();
  checkActiveController(simpleFailProofController_);
}

TEST_F(TestControllerManager, cannotSwitchFromEmergencyControllerToControllerAWithUnclearedEstop) {
  clearEstopAndSwitchController(simpleControllerA_);
  emergencyStop();
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::ERROR, controllerManager_.switchController(simpleControllerB_));
}

TEST_F(TestControllerManager, canSwitchFromEmergencyControllerToControllerA) {
  clearEstopAndSwitchController(simpleControllerA_);
  emergencyStop();
  clearEstopAndSwitchController(simpleControllerB_);
}

TEST_F(TestControllerManager, doesNotSwitchIfControllerAlreadyRunning) {
  clearEstopAndSwitchController(simpleControllerA_);
  ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::RUNNING, controllerManager_.switchController(simpleControllerA_));
  ASSERT_EQ(simpleControllerA_, controllerManager_.getActiveControllerName());
}

TEST_F(TestControllerManager, updatesSuccessfullyFor10ms) {
  clearEstopAndSwitchController(simpleControllerA_);
  runControllerManagerUpdateFor(0.01);
  cancelControllerManagerUpdate();
}

}  // namespace rocoma_test
