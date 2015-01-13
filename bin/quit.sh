#!/bin/bash
screen -S lowlevel_controller -X quit
screen -S state_estimator -X quit
screen -S locomotion_controller -X quit
screen -S locomotion_controller_manager -X quit
rosnode cleanup

