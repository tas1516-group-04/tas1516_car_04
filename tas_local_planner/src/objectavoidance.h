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
    ObjectAvoidance(double wheelbase, double carwidth, tf::TransformListener* tf_);
    double doObstacleAvoidance(double steeringAngle);

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    double radius;
    double yM;

    sensor_msgs::PointCloud laserPoints;

private:
    bool objectInPath(double steeringAngle);
    bool pointInPath(double x, double y, double angle);
    double getNewSteeringAngle(double steeringAngle);


    tf::TransformListener* tf_;
    double wheelbase_;
    double carwidth_;

};

#endif // OBJECTAVOIDANCE_H
