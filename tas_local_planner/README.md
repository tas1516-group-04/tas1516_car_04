Author: Sebastian Eger

## TAS Local Planner
The TAS Local Planner (TLP) is a local planner plugin for the move base.

Based on the global plan the TLP calculates the steering angle for an ackermann vehicle.

### Activating the TLP
To use the TLP you have to set the TLP as the base_local_planner in the move base launch file

<param name="base_local_planner" value="tas_local_planner/LocalPlanner"/>

and load the parameter file for the TLP
<rosparam file="$(find tas)/launch/config/move_base/tas_local_planner_params.yaml" command="load"/>


### How does the TLP work?
TLP is an fairly simple local planner which highly relies on the global plan.

#### A little overview of the compute_velocity function
The compute_velocity function is repeatedly called from the move_base to calculate the cmd_vel data.
First, a target point which fulfills all boundaries (min distance, max sideways distance) is detected.


 


