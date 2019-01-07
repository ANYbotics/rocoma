/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Complex tests to test combinations.
 */

#include <gtest/gtest.h>

#include "rocoma_test/TestControllerManager.hpp"

namespace rocoma_test {

TEST_F(TestControllerManager, parallelSwitch) {
  clearEstopAndSwitchController(simpleControllerA_);
  startSwitchController(sleepyControllerA_);
  usleep(100);
  switchController(simpleControllerB_);
  cancelSwitchController();
}

}  // namespace rocoma_test
