Author: Thomas Eiband
----------------------------------

## Parking

The package "parking" is in charge of the parallel parking task.

To guarantee a good reproducability of the car's trajectories, two criterias for the parking progress were introduced:
1. The car is aligned parallely at a certain distance to the wall during the parking progress, achieved by a P-controller. This controller takes the front laser distance at an angle of 90° to the left side wall as the process variable and the angular velocity as the control input to steer the car parallely along the wall.
2. The backward parking state starts with a fixed steering angle until the change in orientation reaches a threshold, where the orientation is measured by the IMU. After that point, the steering angle is reversed to its maximum and the car drives into the gap.

The package "parking_control" is an adaption of the provided package "tas_autonomous_control". The controlled speeds come directly via the `/cmd_vel` topic from the parking package, described above.


### Start
The parking progress is launched by

`roslaunch parking run.launch`

where the launch file includes the parking parameters, defined in a parameter file.
Additionally, the parking node itself need to be started by

`rosrun parking parking_node`

The execution of this node is seperated from the launch file, such that the restarting of the parking progress can be easily achieved by restarting only this node. The Wii-Mote stays paired and the hardware nodes keep running during this time.

During the parking progress, the C-button of the Wii-Mote need to be hold for safety reasons.

### Parameter Tuning
All parking parameters are defined in the file *params.yaml* inside the folder "parking/launch". These parameters are written to the ROS parameter server on startup and can hence be changed online.
The parameters are stored under the namespace *parking/*

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
| `YAW_THRESHOLD`			| Angular threshold, when car is in the orientation to reverse steering			|
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

### IMU yaw data acquistion

To get an overview about the IMU yaw during the parking progress, a vector of the yaw over the time is created. To save this vector, press 'y' at the end of the parking progress, when requested. The data is then saved to a text file in the home folder of the car.


