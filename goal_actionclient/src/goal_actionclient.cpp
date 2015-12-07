#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_actionclient");
	MoveBaseClient client("move_base", true);
	ROS_INFO("Waiting for action server to come up...");
	client.waitForServer();
	ROS_INFO("Connected to server.");

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 20.0;
	goal.target_pose.pose.position.y = 19.0;
	goal.target_pose.pose.position.z = 0.0;
	
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.999;
	goal.target_pose.pose.orientation.w = 0.0;

	ROS_INFO("Sending goal");
	client.sendGoal(goal);

	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  	if (finished_before_timeout)
  	{
    	actionlib::SimpleClientGoalState state = ac.getState();
    	ROS_INFO("Action finished: %s",state.toString().c_str());
  	}
  	else
  	{
  		actionlib::SimpleClientGoalState state = ac.getState();
    	ROS_INFO("Action did not finish before the time out. Status: %s", state.toString().c_str());
  	}
	
	return 0;
}