Locomotion Controller 
=====================

Run the default locomotion controller with

	roslaunch locomotion_controller locomotion_controller.launch

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

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/starleth_locomotion_controller/issues).


[ROS]: http://www.ros.org
[Eigen]: http://eigen.tuxfamily.org
[Screen]: http://www.gnu.org/software/screen/screen.html