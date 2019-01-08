/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Controller implementation for testing.
 */

#pragma once

#include <roco/controllers/controllers.hpp>

#include "rocoma_test/RocoCommand.hpp"
#include "rocoma_test/RocoState.hpp"

namespace rocoma_test {

class SimpleController : virtual public roco::Controller<rocoma_test::RocoState, rocoma_test::RocoCommand> {
 public:
  using Base = roco::Controller<rocoma_test::RocoState, rocoma_test::RocoCommand>;
  SimpleController() : Base() { setName("SimpleController"); }
  ~SimpleController() override = default;

 protected:
  bool create(double dt) override { return true; }
  bool initialize(double dt) override { return true; }
  bool advance(double dt) override { return true; }
  bool reset(double dt) override { return true; }
  bool preStop() override { return true; }
  bool stop() override { return true; }
  bool cleanup() override { return true; }
};

}  // namespace rocoma_test
