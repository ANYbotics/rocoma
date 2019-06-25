# Robot Controller Manager

## Overview

This software package provides a C++ robot controller manager that is compatible with the [roco](https://github.com/anybotics/roco) interface.

A complete documentation of rocoma is available [here](http://docs.leggedrobotics.com/rocoma_doc/).

The software has been tested under ROS Melodic and Ubuntu 18.04.

The source code is released under a [BSD 3-Clause license](LICENSE).


**Author(s):** Gabriel Hottiger, Christian Gehring


## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

### Dependencies

* **[roco](https://github.com/anybotics/roco):** Robot controller interface
* **[message_logger](https://github.com/anybotics/message_logger):** Logger for messages
* **[signal_logger](https://github.com/anybotics/signal_logger):** Logger for variables
* **[any_common](https://bitbucket.org/leggedrobotics/any_common):** Common tools (workers)

## Usage

An example on how to use this library is given here: [rocoma_example](https://github.com/anybotics/rocoma_example).
