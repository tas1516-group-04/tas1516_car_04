## Parking

The package "parking" is in charge to fullfill the parallel parking task. 

The package "parking_control" is an adaption of the provided package "tas_autonomous_control", where a speed controller is implemented to perform the parking at different velocities.

### Start
The parking progress is launched by

`roslaunch parking run.launch`

where the launch file includes the parking parameters, defined in the parameter file "params.yaml" inside the folder "parking/launch".
The parking node itself is started by

`rosrun parking parking_node`

The execution of this node is seperated from the launch file, that the restarting of the parking progress can be easily achieved by restarting only this node. The Wii-Mote stays paired during this time.

### States
1. *INIT:* Align car parallel to wall with a P-controller
2. *FIRST_CORNER_START:* drive forward until start of first corner is detected (front laser)
3. *FIRST_CORNER_END:* drive forward until end of first corner is detected (front laser)
4. *SECOND_CORNER_START:* drive forward until start of second corner is detected (front laser)
5. *SECOND_CORNER_END:* drive forward until start of second corner is detected (rear laser)
6. *START_PARKING:* start backward parking with steering angle defined in params file, until IMU yaw reaches a threshold
7. *PARKING_STATE_1:* reverse steering angle to maximum and drive further backwards, until rear block comes to close
8. *PARKING_STATE_2:* drive forward until front block comes to close (correction step)
9. *END_PARKING:* the car has reached its final parking position
