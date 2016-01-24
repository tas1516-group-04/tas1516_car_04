#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <laser_geometry/laser_geometry.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <fstream>
#include "objectavoidance.h"

using namespace std;

#ifndef TAS_LOCAL_PLANNER_CPP
#define TAS_LOCAL_PLANNER_CPP

namespace tas_local_planner {

class LocalPlanner : public nav_core::BaseLocalPlanner
{
public:

    LocalPlanner();
    LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseLocalPlanner **/
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    bool isGoalReached();

private:

    // parameters
    double carwidth_;
    double wheelbase_;
    bool doObstacleAvoidance_;
    double minDistance_;
    double steeringAngleParameter_;
    double laserMaxDist_;
    double minObjectSize_;
    double offset_;
    double corridorWidth_;

    // variables
    bool goalIsReached_;
    bool initialized_;
    bool globalPlanIsSet_;

    // ros
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::NodeHandle nodeHandle_;
    tf::TransformListener* tf_;
    std::vector<geometry_msgs::PoseStamped> plan_;

    //obstacle avoidance
    ObjectAvoidance *objectAvoidance;

    //debugging
    int oldPoint;

    // functions
    float calcDistance(geometry_msgs::PoseStamped& a, geometry_msgs::PoseStamped& b);
    double calcAngle(geometry_msgs::PoseStamped point);

};
};
#endif
