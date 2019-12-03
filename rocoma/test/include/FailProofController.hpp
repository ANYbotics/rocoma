/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       FailProof controller implementation for testing.
 */

#pragma once

#include <roco/controllers/FailproofController.hpp>

#include "RocoCommand.hpp"
#include "RocoState.hpp"

namespace rocoma {

class FailProofController : virtual public roco::FailproofController<RocoState, RocoCommand> {
 public:
  using Base = roco::FailproofController<RocoState, RocoCommand>;
  FailProofController() : Base() { setName("FailProofController"); }
  ~FailProofController() override = default;

 protected:
  bool create(double /*dt*/) override { return true; }
  void advance(double /*dt*/) override {}
  bool cleanup() override { return true; }
};

}  // namespace rocoma
