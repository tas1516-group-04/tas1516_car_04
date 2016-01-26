Author: Thomas Eiband
----------------------------------

## Parking

The package "parking" is in charge of the parallel parking task. The code is implemented in `parking_node.cpp`.

To guarantee a good reproducability of the car's trajectories, two criterias for the parking progress were introduced:
1. The car is aligned parallely at a certain distance to the wall during the parking progress, achieved by a P-controller. This controller takes the front laser distance at an angle of 90° to the left side wall as the process variable and the angular velocity as the control input to steer the car parallely along the wall.
2. The backward parking state starts with a fixed steering angle until the change in orientation reaches a threshold, where the orientation is measured by the IMU. After that point, the steering angle is reversed to its maximum and the car drives into the gap.

The package "parking_control" is an adaption of the provided package "tas_autonomous_control". The commanded speeds come directly via the `/cmd_vel` topic from the parking package, described above.


### Start
The hardware for the parking progress is started by

`roslaunch parking run_parking.launch`

The parking node itself need to be started by

`roslaunch parking run_node.launch`

This launch file includes the parking parameters, defined in a dedicated parameter file. The restarting of the parking progress can be easily achieved by restarting only this launch file. The Wii-Mote stays paired and the hardware nodes keep running.

During the parking progress, the C-button of the Wii-Mote need to be hold for safety reasons.

### Parameter Tuning
All parking parameters are defined in the file *params.yaml* inside the folder "parking/launch". These parameters are written to the ROS parameter server on startup and can hence be changed online.
The parameters are stored under the namespace */parking*

 Parameters		     	  	| Comment       									|
| --------------------------------------|-----------------------------------------------------------------------		|
| `MAX_DIST`	                 	| Front laser (at 90°): maximum distance to wall 					|
| `MIN_DIST`			  	| Front laser (at 90°): minimum distanced to wall					|
| `MAX_GAP_DEPTH`			| Front laser (at 90°): maximum parking gap depth					|
| `MIN_GAP_DEPTH`			| Front laser (at 90°): minimum parking gap depth					|
| `FORWARD_THRESHOLD_1`			| Front laser (at  0°): minimum distance to car frontside				|
| `FORWARD_THRESHOLD_2`			| Front laser (at  0°): maximum distance to car frontside				|
| `BACKWARD_THRESHOLD_1`		| Rear laser (at 0°): minimum distance to car backside, to reverse steering angle	|
| `BACKWARD_THRESHOLD_2`		| Rear laser (at 0°): minimum distance to car backside inside parking gap		|
| `LINEAR_SPEED`			| Forward linear parking speed								|
| `ANGULAR_SPEED`			| Angular parking speed									|
| `BACKWARD_SPEED_1`			| 1st Backward linear speed, when initiating backward parking				|
| `BACKWARD_SPEED_2`			| 2nd Backward linear speed, after steering is reversed					|
| `YAW_THRESHOLD`			| Angular threshold, when the car reached its orientation to reverse the steering	|
| `STEERING_FIRST`			| Angular speed, when initiating backward parking					|


### States
1. *INIT:* Align car parallel to the wall with a P-controller
2. *FIRST_CORNER_START:* drive forward until start of the first corner is detected (front laser)
3. *FIRST_CORNER_END:* drive forward until end of the first corner is detected (front laser)
4. *SECOND_CORNER_START:* drive forward until start of the second corner is detected (front laser)
5. *SECOND_CORNER_END:* drive forward until start of the second corner is detected (rear laser)
6. *START_PARKING:* start backward parking with steering angle defined in params file, until IMU yaw reaches a threshold
7. *PARKING_STATE_1:* reverse steering angle to maximum and drive further backwards, until rear block comes close enough
8. *PARKING_STATE_2:* drive forward until front block comes close enough (correction step)
9. *END_PARKING:* the car has reached its final parking position

#### Topics
| Subscribed Topics             | Comment                                                                       |
| ------------------------------|-----------------------------------------------------------------------        |
| `/scan`                     | Front laser scan     |
| `/scan_back`				| Rear laser scan                       |
| `/imu/data			| Imu data to obtain yaw of the car 	|


| Advertised Topics             | Comment                                                                       |
| ------------------------------|-----------------------------------------------------------------------        |
| `/cmd_vel`                         | Linear and angular velocities for longitudinal and lateral control of the car       |



### IMU yaw data acquistion

To get an overview about the IMU yaw during the parking progress, a vector of the yaw over the time is created. To save this vector, press 'y' at the end of the parking progress, when requested. The data is then saved to a text file in the home folder of the car.


Author: Fabian Lechner
----------------------------------

## Adaptive Speed Control
The "speed_control" package implements an adaptive speed controller based on the global path.

The speed_control is divided in two tasks. The 'speed_control.cpp' handles the IO between the speed controller and the ROS system. It subscribes and advertises to all topics necessary for the speed controller to run. The speed controller itself is implemented as a class in 'speed_controller.cpp' and 'speed_controller.h'. Two methods for speed control are available.

### Running the Speed Controller
The speed control is launched by

`roslaunch tas speed_control.launch`

which will load parameters from the parameter server and initialize the speed_controller. The controller will provide velocity commands as soon as a global path is available.

##### Topics
| Subscribed Topics     	| Comment       								|
| ------------------------------|-----------------------------------------------------------------------	|
| `cmd_vel`			| Listening to pose.linear.y as a command to turn the speed control on and off. Sending *y == 1.0* starts the speed control	|
| `move_base_node/NavfnROS/plan`| Global plan, which is used to compute velocity commands			| 

| Advertised Topics	    	| Comment       								|
| ------------------------------|-----------------------------------------------------------------------	|
| `vel`				| A *PoseStamped* message with pose.linear.x set to the computed velocity 	|

##### Parameters
The speed controller is initialized with parameters from the parameter server. All parameters are defined in the file *speed_control.yaml* located in the subdirectory of the launch file.

| Parameters		     	| Comment       								|
| ------------------------------|-----------------------------------------------------------------------	|
| `jump_segments`	        | Amount of segments to leave out in the global path to speed up computation 	|
| `angle_min`			| Minimal accumulated angle allowed for the accumulating angle method		|
| `angle_max`			| Maximal accumulated angle allowed for the accumulating angle method		|
| `short_limit`			| Upper bound for angles between current heading and points on the path with shorth distance|
| `long_limit`			| Upper bound for angles between current heading and points on the path with long distance|
| `short_dist`			| Boundary for points rated as a *short* distance from the car			|
| `long_dist`			| Boundary for points rated as a *long* distance from the car			|
| `min_vel`			| Minimal velocity command provided by the speed controller			|
| `max_vel`			| Maximal velocity command provided by the speed controller			| 


### Velocity Computation
The speed controller tries to control the velocity in a way that imitates natural driving behaviour. This means the velocity should be decreased when entering curves, accounting for the curvature, and the velocity should be increased when leaving cuvy sections. The speed controller will use the global plan to estimate the difficulty of the path ahead and set the speed accordingly. Two methods implementing this behaviour are available.

##### Method 1 - Accumulating angle 
Accumulates all changes in orientation the car would accumulate when traversing the path and calculates a velocity based on the weighted accumulated angle. 

1. Transform path into the base_link frame of the car.
2. Traverse the path up to a maximum distance and accumulate all angles that exist between the individual path segments.
3. Weight every angle depending on its distance to the current position. Curves occuring in a large distance will have less of an effect than curves directly in front of the car.
4. Bound the weight in the interval [0,1] and map the weight to the interval [max_vel, min_vel]
5. Repeat for new path.

##### Method 2 - Angles at fixed points in the path
Points with fixed distance to the car are set in the path. As the car traverses through the path, the points remain a fixed distance from the car following the shape of the path. The relative orientation between *point to car* and *current heading* can be used to compute velocity commands.

1. Transform path into the base_link frame of the car.
2. Traverse the path and set points at fixed distances from the car.
3. Compute the angles between the lines point->car and current heading and weigh depending on distance.
4. Bound the weight in the interval [0,1] and map the weight to the interval [max_vel, min_vel]
5. Repeat for new path.



 


