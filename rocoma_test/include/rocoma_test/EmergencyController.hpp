/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Emergency controller implementation for testing.
 */

#pragma once

#include <roco/controllers/controllers.hpp>

#include "rocoma_test/RocoCommand.hpp"
#include "rocoma_test/RocoState.hpp"

namespace rocoma_test {

class EmergencyController : virtual public roco::Controller<RocoState, RocoCommand>, public roco::EmergencyControllerAdapteeInterface {
 public:
  using Base = roco::Controller<RocoState, RocoCommand>;
  EmergencyController() : Base() { setName("EmergencyController"); }
  ~EmergencyController() override = default;

 protected:
  bool create(double dt) override { return true; }
  bool initialize(double dt) override { return true; }
  bool advance(double dt) override { return true; }
  bool reset(double dt) override { return true; }
  bool preStop() override { return true; }
  bool stop() override { return true; }
  bool cleanup() override { return true; }

  bool initializeFast(double dt) override { return true; }
};

}  // namespace rocoma_test
