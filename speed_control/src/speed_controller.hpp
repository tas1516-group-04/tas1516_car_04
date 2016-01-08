#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <cmath>

class SpeedController
{
public:
    SpeedController(const tf::TransformListener*);
    size_t locateOnPath(nav_msgs::PathConstPtr);
    double calcCurveValue();
    double calcSpeed();
    void planReceivedCallback(const nav_msgs::Path::ConstPtr &path);
    nav_msgs::Path::ConstPtr current_plan;
private:
    const tf::TransformListener* tf_listener;

    inline double calcDistance(const geometry_msgs::PoseStamped& pose1,const geometry_msgs::PoseStamped& pose2)
    {
        return sqrt(pow((pose1.pose.position.x - pose2.pose.position.x), 2) + pow((pose1.pose.position.y - pose2.pose.position.y), 2));
    }
    inline double calcAngle(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
    {
        return atan2(pose1.pose.position.y - pose2.pose.position.y, pose1.pose.position.x - pose2.pose.position.x);
    }
    void getCarPose(geometry_msgs::PoseStamped& carPose)
    {
        tf::StampedTransform carTransform;
        tf_listener->lookupTransform("map", "base_link", ros::Time(0), carTransform);
        carPose.pose.position.x = carTransform.getOrigin().getX();
        carPose.pose.position.y = carTransform.getOrigin().getY();
        carPose.pose.orientation.w = carTransform.getRotation().getW();
        carPose.pose.orientation.x = carTransform.getRotation().getX();
        carPose.pose.orientation.y = carTransform.getRotation().getY();
        carPose.pose.orientation.z = carTransform.getRotation().getZ();
    }
};

SpeedController::SpeedController(const tf::TransformListener* listener)
{
    this->tf_listener = listener;
}
void SpeedController::planReceivedCallback(const nav_msgs::Path::ConstPtr& path)
{
    current_plan = path;
}
size_t SpeedController::locateOnPath(nav_msgs::PathConstPtr current_path)
{
    // Find the position of the car in map and transform it to PoseStamped
    tf::StampedTransform carTransform;
    tf_listener->lookupTransform("map", "base_link", ros::Time(0), carTransform);
    geometry_msgs::PoseStamped carPose;
    getCarPose(carPose);


    double minDistance = calcDistance((geometry_msgs::PoseStamped&) current_path->poses[0], carPose);
    size_t closestPointIdx = 0;
    for (size_t i = 1; i < current_path->poses.size(); i += 5)
    {
        double distance = calcDistance((geometry_msgs::PoseStamped&) current_path->poses[i], carPose);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestPointIdx = i;
        }
    }
    return closestPointIdx;
}

#endif // SPEED_CONTROLLER_HPP
