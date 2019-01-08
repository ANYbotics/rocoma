/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Controller implementation with sleeps for testing.
 */

#pragma once

#include <roco/controllers/controllers.hpp>

#include "rocoma_test/RocoCommand.hpp"
#include "rocoma_test/RocoState.hpp"

namespace rocoma_test {

class SleepyController : virtual public roco::Controller<rocoma_test::RocoState, rocoma_test::RocoCommand> {
 public:
  using Base = roco::Controller<rocoma_test::RocoState, rocoma_test::RocoCommand>;
  SleepyController() : Base() { setName("SleepyController"); }
  ~SleepyController() override = default;

 protected:
  bool create(double dt) override { return true; }
  bool initialize(double dt) override {
    usleep(sleepDuration_);
    return true;
  }
  bool advance(double dt) override { return true; }
  bool reset(double dt) override {
    usleep(sleepDuration_);
    return true;
  }
  bool preStop() override {
    usleep(sleepDuration_);
    return true;
  }
  bool stop() override {
    usleep(sleepDuration_);
    return true;
  }
  bool cleanup() override { return true; }

 private:
  static constexpr int sleepDuration_ = 5000;
};

}  // namespace rocoma_test
