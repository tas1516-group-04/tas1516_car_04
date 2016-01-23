## Parking

The package "parking" is in charge of the parallel parking task. 

The package "parking_control" is an adaption of the provided package "tas_autonomous_control", where a speed controller is implemented to perform the parking at different velocities.

### Start
The parking progress is launched by

`roslaunch parking run.launch`

where the launch file includes the parking parameters, defined in the parameter file *params.yaml* inside the folder "parking/launch".
Additionally, the parking node itself need to be started by

`rosrun parking parking_node`

The execution of this node is seperated from the launch file, such that the restarting of the parking progress can be easily achieved by restarting only this node. The Wii-Mote stays paired and the hardware nodes keep running during this time.

### Parameter Tuning
As mentioned before, all parking parameters are defined in the file *params.yaml*. These parameters are written to the ROS parameter server on startup and can hence be changed online, even during the parking progress.
The parameters are stored under the namespace *parking/*

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

### IMU yaw data acquistion

To get an overview about the IMU yaw during the parking progress, a vector of the yaw over the time is created. To save this vector, press 'y' at the end of the parking progress, when requested. The data is then saved to a text file in the home folder of the car.


