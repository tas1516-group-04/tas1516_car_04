#ifndef OBJECTAVOIDANCE_H
#define OBJECTAVOIDANCE_H

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

class ObjectAvoidance
{
public:
    ObjectAvoidance(double wheelbase,
                    double carwidth,
                    double corridorWidth,
                    double minDistance);
    geometry_msgs::PoseStamped doObstacleAvoidance(int targetPoint, std::vector<geometry_msgs::PoseStamped> plan, geometry_msgs::Twist& cmd_vel);

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    double minObjectSize_;
private:

    // parameters
    double wheelbase_;
    double carwidth_;
    double corridorWidth_;
    double minDistance_;

    // node handle
    ros::NodeHandle nodeHandle_;
    ros::Subscriber subScan_;
    ros::Publisher pubScanTf_;

    // variables
    double distToPoint_;
    sensor_msgs::LaserScan::ConstPtr scan_;
    std::vector<geometry_msgs::Point32> laserPoints;

    // functions
    void filterLaserScan();
    bool objectInPath(geometry_msgs::PoseStamped targetPoint);
    bool pointInPath(double x, double y, geometry_msgs::PoseStamped targetPoint);
    geometry_msgs::PoseStamped getNewTargetPoint(geometry_msgs::PoseStamped targetPoint);

};

#endif // OBJECTAVOIDANCE_H
