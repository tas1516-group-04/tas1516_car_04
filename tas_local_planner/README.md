Author: Sebastian Eger
----------------------------------

## TAS Local Planner
The TAS Local Planner (TLP) is a local planner plugin for the move base.

Based on the global plan the TLP calculates the steering angle for an ackermann vehicle.
If obstacle avoidance is active, TLP also detects objects in the path and tries to avoid them.

More about plugins for ROS
`http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS`


### Activating the TLP
To use the TLP you have to set the TLP as the base_local_planner in the move base launch file

`<param name="base_local_planner" value="tas_local_planner/LocalPlanner"/>`

and load the parameter file for the TLP

`<rosparam file="$(find tas)/launch/config/move_base/tas_local_planner_params.yaml" command="load"/>`

### Parameters
 Parameters		     	  	          | Comment       								|
| ------------------------------------|-----------------------------------------------------------------------	|
| `car_width`	                 	  | Width of the used car 	| 
| `wheelbase`                         | Distance between front and rear axis		|
| `corridor_width`                    | Size of the corridor for target point search	|
| `obstacle_avoidance`                | If true, obstacle avoidance is active|
| `min_distance`                      | Minimum distance of the target point|
| `steering_angle_parameter`          | Increases or decrease steering angle by a percentage		|
| `min_object_size`			          | Minimum size of a detected object|
| `offset`				              | Offset in radians		|



 


