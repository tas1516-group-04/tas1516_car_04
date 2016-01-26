Author: Fabian Lechner

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



 


