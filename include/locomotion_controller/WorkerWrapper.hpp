/*
 * WorkerWrapper.hpp
 *
 *  Created on: Jun 18, 2015
 *      Author: dario
 */

#pragma once

#include <roscpp_nodewrap/worker/Worker.h>
#include <roscpp_nodewrap/worker/WorkerEvent.h>
#include <roco/workers/WorkerOptions.hpp>
#include <roco/workers/WorkerEvent.hpp>

class WorkerWrapper
{
 public:
  inline bool workerCallback(const nodewrap::WorkerEvent& workerEvent)
  {
    roco::WorkerEvent event;
    return options_.callback_(event);
  }
  roco::WorkerOptions options_;
  nodewrap::Worker worker_;
};
