#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <fstream>

using std::string;

#ifndef TAS_LOCAL_PLANNER_CPP
#define TAS_LOCAL_PLANNER_CPP

// distance between the front and back wheels
#define WHEELBASE 0.3
// car width
#define CARWIDTH  0.5

sensor_msgs::LaserScan::ConstPtr tlpLaserScan;

enum LaserObjectType {
    wall = 0,
    obstacle = 1
};

struct LaserObject {
    LaserObjectType type;
    int start;
    int end;
};

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

    //variables
    bool goalIsReached_;
    bool initialized_;
    bool globalPlanIsSet_;
    tf::TransformListener* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::NodeHandle nodeHandle_;
    ros::Subscriber subScan_;
    ros::Publisher pubTest_;
    std::vector<geometry_msgs::PoseStamped> plan_;
    geometry_msgs::PoseStamped robotPose_;

    //laser
    std::vector<geometry_msgs::Pose> laserDataTf_;
    std::vector<LaserObject> laserObjects_;

    //debug
    std::ofstream debugFile_;


    // functions
    //void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    int makeDecision();
    void analyzeLaserData();
    void tfRobotPose();

};
};
#endif
