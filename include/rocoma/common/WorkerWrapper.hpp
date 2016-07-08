/*
 * WorkerWrapper.hpp
 *
 *  Created on: Jun 18, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <any_worker/Worker.hpp>
#include <any_worker/WorkerEvent.hpp>

#include <roco/workers/WorkerOptions.hpp>
#include <roco/workers/WorkerEventInterface.hpp>
#include <roco/workers/WorkerEventStd.hpp>

namespace rocoma {

class WrapperWorkerEvent : public roco::WorkerEventStd, public any_worker::WorkerEvent {
public:
  WrapperWorkerEvent() = delete;
  WrapperWorkerEvent(const any_worker::WorkerEvent& event ):
    any_worker::WorkerEvent(event)
  {

  }

  virtual ~WrapperWorkerEvent()
  {

  }
};

class WorkerWrapper
{
 public:
  WorkerWrapper() = delete;

  WorkerWrapper(roco::WorkerOptions options):
    options_(options)
  {

  }

  virtual ~WorkerWrapper()
  {

  }

  inline bool workerCallback(const any_worker::WorkerEvent& workerEvent)
  {
    WrapperWorkerEvent event(workerEvent);
    return options_.callback_(event);
  }

 private:
  roco::WorkerOptions options_;
};

}
