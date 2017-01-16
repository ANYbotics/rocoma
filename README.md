# Robot Controller Manager

## Overview

This software package provides a controller which enables ANYmal to locomote with a trotting gait.

A complete documentation of rocoma is available [here](http://docs.leggedrobotics.com/rocoma_doc/).

The software has been tested under ROS Indigo and Ubuntu 14.04.

The source code is released under a [BSD 3-Clause license](LICENSE).


**Author(s):** Gabriel Hottiger, Christian Gehring


## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

### Dependencies

* **[roco](https://bitbucket.org/leggedrobotics/roco):** Robot controller interface
* **[message_logger](https://bitbucket.org/leggedrobotics/message_logger):** Logger for messages
* **[signal_logger](https://bitbucket.org/leggedrobotics/signal_logger):** Logger for variables
* **[any_common](https://bitbucket.org/leggedrobotics/any_common):** Common tools (workers)
	
## Usage

An example on how to use this library is given here: [rocoma_example](https://bitbucket.org/leggedrobotics/rocoma_example).


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).
