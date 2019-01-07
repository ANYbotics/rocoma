/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Test fixture for testing the controller manager.
 */

#pragma once

#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <rocoma/ControllerManager.hpp>
#include <rocoma/controllers/adapters.hpp>

#include "rocoma_test/EmergencyController.hpp"
#include "rocoma_test/FailProofController.hpp"
#include "rocoma_test/SimpleController.hpp"
#include "rocoma_test/SleepyController.hpp"

namespace rocoma_test {

class TestControllerManager : public ::testing::Test {
 protected:
  using SimpleCtrl = rocoma::ControllerAdapter<SimpleController, RocoState, RocoCommand>;
  using SleepyCtrl = rocoma::ControllerAdapter<SleepyController, RocoState, RocoCommand>;
  using EmergencyCtrl = rocoma::EmergencyControllerAdapter<EmergencyController, RocoState, RocoCommand>;
  using FailProofCtrl = rocoma::FailproofControllerAdapter<FailProofController, RocoState, RocoCommand>;

 public:
  TestControllerManager()
      : timeStep_(0.1),
        controllerManager_(),
        state_(new RocoState()),
        command_(new RocoCommand()),
        mutexState_(new boost::shared_mutex()),
        mutexCommand_(new boost::shared_mutex()),
        updateThread_{} {
    setupSimpleControllerManager();
    setupSimpleControllers();
  }

  void setupSimpleControllers() {
    //! Failproof controller
    std::unique_ptr<FailProofCtrl> controllerFailProof(new FailProofCtrl());
    controllerFailProof->setName(simpleFailProofController_);
    controllerFailProof->setParameterPath(simpleFailProofController_ + "/Parameters.json");
    controllerFailProof->setStateAndCommand(state_, mutexState_, command_, mutexCommand_);
    controllerManager_.setFailproofController(std::move(controllerFailProof));

    //! Emergency controller
    std::unique_ptr<EmergencyCtrl> controllerEmergency(new EmergencyCtrl());
    controllerEmergency->setName(simpleEmergencyController_);
    controllerEmergency->setStateAndCommand(state_, mutexState_, command_, mutexCommand_);
    controllerEmergency->setParameterPath(simpleEmergencyController_ + "/Parameters.yaml");

    //! Setup controller A
    std::unique_ptr<SimpleCtrl> controllerA(new SimpleCtrl());
    controllerA->setName(simpleControllerA_);
    controllerA->setStateAndCommand(state_, mutexState_, command_, mutexCommand_);
    controllerA->setParameterPath(simpleControllerA_ + "/Parameters.xml");
    controllerManager_.addControllerPair(std::move(controllerA), nullptr);

    //! Setup controller B
    std::unique_ptr<SimpleCtrl> controllerB(new SimpleCtrl());
    controllerB->setName(simpleControllerB_);
    controllerB->setStateAndCommand(state_, mutexState_, command_, mutexCommand_);
    controllerB->setParameterPath(simpleControllerB_ + "/Parameters.xml");
    controllerManager_.addControllerPair(std::move(controllerB), std::move(controllerEmergency));

    //! Setup sleepy controller A
    std::unique_ptr<SleepyCtrl> sleepyControllerA(new SleepyCtrl());
    sleepyControllerA->setName(sleepyControllerA_);
    sleepyControllerA->setStateAndCommand(state_, mutexState_, command_, mutexCommand_);
    sleepyControllerA->setParameterPath(sleepyControllerA_ + "/Parameters.xml");
    controllerManager_.addControllerPair(std::move(sleepyControllerA), nullptr);
  }

  void setupSimpleControllerManager() {
    rocoma::ControllerManagerOptions managerOptions;
    managerOptions.isRealRobot = false;
    managerOptions.timeStep = timeStep_;
    managerOptions.emergencyStopMustBeCleared = true;
    managerOptions.loggerOptions.enable = true;
    managerOptions.loggerOptions.fileTypes = {signal_logger::LogFileType::CSV};
    managerOptions.loggerOptions.updateOnStart = true;
    controllerManager_.init(managerOptions);
  }

  void failproofStop() {
    ASSERT_TRUE(controllerManager_.failproofStop());
    controllerManager_.stopWorkers();  // Joins stop threads
  }

  void emergencyStop() {
    ASSERT_TRUE(controllerManager_.emergencyStop());
    controllerManager_.stopWorkers();  // Joins stop threads
  }

  void switchController(const std::string& controllerName) {
    ASSERT_EQ(rocoma::ControllerManager::SwitchResponse::SWITCHING, controllerManager_.switchController(controllerName));
    ASSERT_EQ(controllerName, controllerManager_.getActiveControllerName());
  }

  void startSwitchController(const std::string& controllerName) {
    //! Join old thread, but warn user
    if (switchThread_.joinable()) {
      MELO_WARN("[TestControllerManager] Old switch thread is still running. Try to join it.");
      switchThread_.join();
    }
    switchThread_ =
        std::thread([this, controllerName]() { controllerManager_.switchController(controllerName, std::ref(switchPromise_)); });
  }

  rocoma::ControllerManager::SwitchResponse cancelSwitchController() {
    std::future<rocoma::ControllerManager::SwitchResponse> f = switchPromise_.get_future();
    auto result = f.get();
    switchThread_.join();
    return result;
  }

  void clearEstopAndSwitchController(const std::string& controllerName) {
    controllerManager_.clearEmergencyStop();
    switchController(controllerName);
  }

  void checkActiveController(const std::string& controllerName) { ASSERT_EQ(controllerName, controllerManager_.getActiveControllerName()); }

  void runControllerManagerUpdateFor(double seconds) {
    //! Join old thread, but warn user
    if (updateThread_.joinable()) {
      MELO_WARN("[TestControllerManager] Old update thread is still running. Try to join it.");
      updateThread_.join();
    }
    updateThread_ = std::thread(&TestControllerManager::updateControllerManagerWorker, this, seconds);
  }

  void cancelControllerManagerUpdate() {
    if (updateThread_.joinable()) {
      updateThread_.join();
    }
  }

 private:
  void updateControllerManagerWorker(double seconds) {
    boost::asio::io_service io;
    unsigned int counter = 0;
    while ((counter * timeStep_) < seconds) {
      boost::asio::deadline_timer t(io, boost::posix_time::milliseconds(timeStep_ * 1000));
      ASSERT_TRUE(controllerManager_.updateController());
      t.wait();
      counter++;
    }
  }

 protected:
  // Controller manager data
  double timeStep_;
  rocoma::ControllerManager controllerManager_;
  std::shared_ptr<RocoState> state_;
  std::shared_ptr<RocoCommand> command_;
  std::shared_ptr<boost::shared_mutex> mutexState_;
  std::shared_ptr<boost::shared_mutex> mutexCommand_;

  // Update thread
  std::thread updateThread_;
  std::thread switchThread_;
  std::promise<rocoma::ControllerManager::SwitchResponse> switchPromise_;

  //! Controller names
  const std::string simpleFailProofController_ = std::string{"SimpleFailProofController"};
  const std::string simpleEmergencyController_ = std::string{"SimpleEmergencyController"};
  const std::string simpleControllerA_ = std::string{"SimpleControllerA"};
  const std::string simpleControllerB_ = std::string{"SimpleControllerB"};
  const std::string sleepyControllerA_ = std::string{"SleepyControllerA"};
  const std::string sleepyControllerB_ = std::string{"SleepyControllerA"};
};

}  // namespace rocoma_test
