/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Roco command implementation for testing.
 */

#pragma once

#include <message_logger/message_logger.hpp>
#include <roco/model/CommandInterface.hpp>

namespace rocoma_test {

class RocoCommand : public roco::CommandInterface {
 public:
  bool limitCommand() override {
    if (value_ > maxValue_) {
      MELO_INFO("[rocoma_test::RocoCommand]: Limiting Command Event. Value %f bigger than maximum allowed %f", value_, maxValue_);
      value_ = maxValue_;
    }

    return true;
  }

  void setValue(const double value) { value_ = value; }
  double getValue() const { return value_; }

  static constexpr double maxValue_ = 2.0;

 private:
  double value_;
};

}  // namespace rocoma_test
