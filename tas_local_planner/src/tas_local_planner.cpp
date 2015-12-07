#include <pluginlib/class_list_macros.h>
#include "tas_local_planner.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(tas_local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

 using namespace std;

 void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
   int number = sizeof(scan->ranges);
   ROS_INFO("Laser ranges: %i", number);
   tlpLaserScan = scan;
}
 //Default Constructor
 namespace tas_local_planner {

 LocalPlanner::LocalPlanner (){
 }

 LocalPlanner::LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, tf, costmap_ros);
 }

 void LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
   if(!initialized_) {
     subScan_ = nodeHandle_.subscribe("scan", 1000, scanCallback);
     tf_ = tf;
     costmap_ros_ = costmap_ros_;
     goalIsReached_ = false;

     //finish initialization
     initialized_ = true;
   } else {
     ROS_INFO("TAS LocalPlanner is already initialized!");
   }
 }

 bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    //ros::spinOnce();
    ROS_INFO("Range 320: %f", tlpLaserScan->ranges[320]);
    if(tlpLaserScan->ranges[320] > 0.5) {
        cmd_vel.linear.x = 0.5;
	return true;
    } else {
        cmd_vel.linear.x = 0;
	return true;
    }
 }
 bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
   plan_ = plan;
   return true;
 }
 bool LocalPlanner::isGoalReached() {
   return goalIsReached_;
 }


 };
