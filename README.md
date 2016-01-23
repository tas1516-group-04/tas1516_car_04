# Parking

The package "parking" is in charge to fullfill the parallel parking task. 

The package "parking_control" is an adaption of the provided package "tas_autonomous_control", where a speed controller is implemented to perform the parking at different velocities.

## Start
The parking progress is launched by

`roslaunch parking run.launch`

where the launch file includes the parking parameters, defined in the parameter file "params.yaml" inside the folder "parking/launch".
The parking node itself is started by

`rosrun parking parking_node`

The execution of this node is seperated from the launch file, that the restarting of the parking progress can be easily achieved by restarting only this node. The Wii-Mote stays paired during this time.

