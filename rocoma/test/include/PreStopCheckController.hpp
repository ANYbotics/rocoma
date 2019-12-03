/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Controller implementation with allocations for testing.
 */

#pragma once

#include <ros/ros.h>

#include <roco/controllers/controllers.hpp>

#include "RocoCommand.hpp"
#include "RocoState.hpp"

namespace rocoma {

class PreStopCheckController : virtual public roco::Controller<RocoState, RocoCommand> {
 public:
  using Base = roco::Controller<RocoState, RocoCommand>;
  PreStopCheckController() : Base() { setName("PreStopCheckController"); }
  ~PreStopCheckController() override = default;

 protected:
  bool create(double /*dt*/) override { return true; }
  bool initialize(double /*dt*/) override {
    getCommand().setValue(0.0);
    return true;
  }
  bool advance(double /*dt*/) override { return true; }
  bool reset(double dt) override { return initialize(dt); }
  bool preStop() override {
    getCommand().setValue(getCommand().getValue() + 1.0);
    return true;
  }
  bool stop() override { return true; }
  bool cleanup() override { return true; }
};

}  // namespace rocoma
