Author: Sebastian Eger

## TAS Local Planner
The TAS Local Planner (TLP) is a local planner plugin for the move base.

Based on the global plan the TLP calculates the steering angle for an ackermann vehicle.

### Activating the TLP
To use the TLP you have to set the TLP as the base_local_planner in the move base launch file

`<param name="base_local_planner" value="tas_local_planner/LocalPlanner"/>`

and load the parameter file for the TLP

`<rosparam file="$(find tas)/launch/config/move_base/tas_local_planner_params.yaml" command="load"/>`

 Parameters		     	  	| Comment       								|
| ------------------------------------|-----------------------------------------------------------------------	|
| `car_width`	                 	| Width of the used car 	|
| `wheelbase`			  	| Distance between the two axis		|
| `corridor_width`			| Size of the corridor for target point search	|
| `obstacle_avoidance`			| If true, obstacle is active|
| `min_distance`			| Minimum distance of the target point|
| `steering_angle_parameter`		| Increases or decrease steering angle by a percentage		|
| `min_object_size`			| Minimum size of a detected object|
| `offset`				| Offset in radians		|



 


