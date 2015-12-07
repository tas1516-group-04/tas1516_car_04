#include <pluginlib/class_list_macros.h>
#include "tas_local_planner.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(tas_local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

 using namespace std;

 //Default Constructor
 namespace tas_local_planner {

 LocalPlanner::LocalPlanner (){
 }

 LocalPlanner::LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, tf, costmap_ros);
 }

 void LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
   if(!initialized_) {
     tf_ = tf;
     costmap_ros_ = costmap_ros_;
     goalIsReached_ = false;
     initialized_ = true;
   } else {
     ROS_INFO("TAS LocalPlanner is already initialized!");
   }
 }

 bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    ROS_INFO("Compute velocity command!");
 }
 bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {

 }
 bool LocalPlanner::isGoalReached() {
   return goalIsReached_;
 }
 };
