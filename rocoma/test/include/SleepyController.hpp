/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Controller implementation with sleeps for testing.
 */

#pragma once

#include <roco/controllers/controllers.hpp>

#include "RocoCommand.hpp"
#include "RocoState.hpp"

namespace rocoma {

class SleepyController : virtual public roco::Controller<RocoState, RocoCommand> {
 public:
  using Base = roco::Controller<RocoState, RocoCommand>;
  SleepyController() : Base() { setName("SleepyController"); }
  ~SleepyController() override = default;

 protected:
  bool create(double /*dt*/) override { return true; }
  bool initialize(double /*dt*/) override {
    usleep(sleepDuration_);
    return true;
  }
  bool advance(double /*dt*/) override { return true; }
  bool reset(double /*dt*/) override {
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

}  // namespace rocoma
