Locomotion Controller for StarlETH
============================

[![Build Status](http://129.132.38.183:8080/buildStatus/icon?job=starleth_locomotion_controller)](http://129.132.38.183:8080/view/legged_robotics/job/starleth_locomotion_controller/)

Overview
---------------
This C++ software package provides a locomotion controller for the quadruped robot StarlETH.

**Author: Christian Gehring**

**Contact: gehrinch@ethz.ch**

**Affiliation: Autonomous Systems Lab, ETH Zurich**



Usage
------------
### Boot-Up Configuration on StarlETH

When the machine *starleth-lpc* boots up, the locomotion controller is started automatically as a *deamon*. 
To **start the deamon** manually, run

```
sudo service starleth-locomotion-controller start
```
The service will wait a few seconds until it starts the locomotion controller, because during boot-up some devices take some time to be ready.
Note that the controller will be started with a homing procedure, i.e. the legs need to be at the hard stops.

To **re-start the locomotion controller**, you can also use the following command:

```
sudo service starleth-locomotion-controller restart
```
This service will stop the locomotion controller if it is still running and start it immediately.

To  **re-start the locomotion controller without homing**, run
```
sudo service starleth-locomotion-controller restart-no-home
```

To **stop the  locomotion controller**, run
```
sudo service starleth-locomotion-controller stop
```

To **check if the  locomotion controller is running**, run
```
sudo service starleth-locomotion-controller status
```


### ROS Launch Configuration on StarlETH
To **start the locomotion controller**, run 

```
roslaunch starleth_locomotion_controller starleth_nodelet_screen.launch
```

This will start the locomotion controller with [screen](http://www.gnu.org/software/screen/screen.html), which is  highly recommended!
Screen prevents stopping the locomotion controller when the user's terminal is closed. 

To **display the output** of the locomotion controller, run 
```
rosrun starleth_locomotion_controller spawn_locomotion_controller
```
 or use screen directly: 
```
screen -r locomotion_controller_manager
```

To **terminate** the locomotion controller, press ```Ctrl + C``` in the shell or run 
```
rosrun starleth_locomotion_controller quit_screen
```
press ```y``` and  ```Enter```.

Check with *screen* and *rosnode* if everything closed properly, but wait about 5 seconds after terminating the locomotion controller:
```
screen -ls
rosnode list
```
If *locomotion_controller_manager*, *locomotion_controller*, *state_estimator* or *lowlevel_controller* appears in the list, re-run
```
rosrun starleth_locomotion_controller quit_screen
```
and check again.

If *locomotion_controller_manager*, *locomotion_controller*, *state_estimator* or *lowlevel_controller* appears *twice* in the list, run 
```
rosrun starleth_locomotion_controller quit_screen_all_detached
```
This will quit all detached screen windows.



### Debugging Configuration On StarlETH

To debug the locomotion controller with [GDB](http://www.gnu.org/software/gdb/), run
```
roslaunch starleth_locomotion_controller starleth_nodelet_gdb.launch
```
Note that this configuration does not run with [screen](http://www.gnu.org/software/screen/screen.html)! 







## ROS Node

#### Subscribed Topics

* **`robot_state`** ([starleth_msgs/RobotState])

    The robot state.

* **`joy`** ([sensor_msgs/Joy])

    The joystick commands.
    
* **`command_velocity`** ([geometry_msgs/Twist])

    The desired command velocity of the robot:
    
        * Heading: linear.x
        * Lateral: linear.y
        * Turning: angular.z 

#### Published Topics

* **`command_seactuators`** ([starleth_msgs/SeActuatorCommands])

    Series-elastic actuator commands.

#### Advertised Services

* **`switch_controller`** ([locomotion_controller_msgs/SwitchController])

    Switches the locomotion controller.
    
* **`emergency_stop`** ([locomotion_controller_msgs/EmergencyStop])

    Executes an emergency stop.

#### Provided Services

* **`reset_state_estimator`** ([locomotion_controller_msgs/ResetStateEstimator])

    Resets the state estimator.
    
#### Parameters


* **`controller/time_step`** (double, default: "0.0025")
 
    Time step in seconds between two consecutive controller updates.

* **`controller/is_real_robot`** (bool, default: "false")
 
	Indicates if the real robot is in the loop. 

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/ethz-asl-lr/starleth_locomotion_controller/issues).


[ROS]: http://www.ros.org
[Eigen]: http://eigen.tuxfamily.org
[Screen]: http://www.gnu.org/software/screen/screen.html