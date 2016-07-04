/*
 * WorkerWrapper.hpp
 *
 *  Created on: Jun 18, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <roscpp_nodewrap/worker/Worker.h>
#include <roscpp_nodewrap/worker/WorkerEvent.h>
#include <roco/workers/WorkerOptions.hpp>
#include <roco/workers/WorkerEventInterface.hpp>
#include <roco/workers/WorkerEventStd.hpp>


class WrapperWorkerEvent : public roco::WorkerEventStd, public nodewrap::WorkerEvent {
public:
  WrapperWorkerEvent() {

  }

  WrapperWorkerEvent(const nodewrap::WorkerEvent& event ): nodewrap::WorkerEvent(event)  {
    actualCycleTime_.from(event.actualCycleTime.sec, event.actualCycleTime.nsec);
    expectedCycleTime_.from(event.expectedCycleTime.sec, event.expectedCycleTime.nsec);
  }

  virtual ~WrapperWorkerEvent() {

  }
  /** \brief The expected cycle time of the worker
    */
  virtual roco::time::Time& getExpectedCycleTime() {
    return actualCycleTime_;
  }

  /** \brief The momentary, actual cycle time of the worker
    */
  virtual roco::time::Time& getActualCycleTime() {
    return expectedCycleTime_;
  }


};

class WorkerWrapper
{
 public:
  inline bool workerCallback(const nodewrap::WorkerEvent& workerEvent)
  {
    WrapperWorkerEvent event(workerEvent);
    return options_.callback_(event);
  }
  roco::WorkerOptions options_;
  nodewrap::Worker worker_;
};
