// Author: Fabian Lechner

#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP

#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <cmath>
#include <vector>
#include <fstream>

class SpeedController {
public:
    SpeedController(const tf::TransformListener *);    

    // Called when a new global plan is published
    void planCallback(const nav_msgs::Path::ConstPtr &);

    // Set parameters from parameter server
    void set_parameters(ros::NodeHandle& node_handle);

    // Compute a velocity and return it as a cmd_vel value for the speed in x direction
    double calcSpeed();

private:
    // Variables for transforming and storing a new global plan
    bool plan_valid;
    const tf::TransformListener *transform_listener;
    std::vector<geometry_msgs::PoseStamped> current_path;

    // Parameters loaded from the parameter server
    int jump_segments;
    double angle_min, angle_max;
    double short_limit, long_limit;
    double short_dist, long_dist;
    double min_vel, max_vel;

    // Transform a path from map frame to base_link frame
    void transformPath(std::vector<geometry_msgs::PoseStamped> &);

    // Euclidean distance from two poses or from two 2D-vectors
    double calcDistance(const geometry_msgs::PoseStamped &, const geometry_msgs::PoseStamped &);
    double calcDistance(const double &x_diff, const double &y_diff);
    // Angle between pose2-pose1 and x-axis(base_link-frame)
    double calcAngle(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);

    // Force a value between lower and upper bounds
    double clip(double n, double lower, double upper);

    // Sum change of angles up to a maximum distance or to the end of the path (Details see .cpp)
    double calcCurveWeight_accumulatingAngle(const double maxDist);

    // Compute velocity from angles between fixed points on path and current heading (Details see .cpp)
    double calcCurveWeight_fixedPoints();

    // Locate the car on the path
    size_t locateOnPath(nav_msgs::PathConstPtr current_path);
};


#endif // SPEED_CONTROLLER_HPP
