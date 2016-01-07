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
//    SpeedController(ros::NodeHandle &nh, const tf::TransformListener *tf);
    SpeedController(ros::NodeHandle &nh, const tf::TransformListener *tf) : node_handle(nh), transform_listener(tf)
    {}

    // Return a cmd_vel value for the speed in x direction
    double calcSpeed();

    // Load parameters from server and start subsciber
    void initialize()
    {
        ROS_INFO("Subscribed to NavfnROS");
        node_handle.param<int>("/speed_control_node/jump_segments", jump_segments, 5);
        node_handle.param<double>("/speed_control_node/angle_min", angle_min, 0.0);
        node_handle.param<double>("/speed_control_node/angle_max", angle_max, 90.0);

        plan_subscriber = node_handle.subscribe("move_base_node/NavfnROS/plan", 10,
                                                    &SpeedController::planCallback, this);

        plan_valid = false;
    }

    void testPath()
    {
        for(auto& x : transformed_path)
        {
            std::cout << x.pose.position.x << std::endl;
        }
    }

private:
    ros::NodeHandle& node_handle;
    ros::Subscriber plan_subscriber;
    bool plan_valid;
    const tf::TransformListener *transform_listener;
    const tf::TransformListener tf_listener;
    std::vector<geometry_msgs::PoseStamped> current_path;
    std::vector<geometry_msgs::PoseStamped> transformed_path;

    // Parameters from the parameter-server
    int jump_segments;
    double angle_min;
    double angle_max;

    // Called when a new global plan is published
    void planCallback(const nav_msgs::Path::ConstPtr &path)
    {
        // Assume valid plan
        plan_valid = true;
        // Need at least some elements in path
//        if (path->poses.size() < jump_segments+15)
//        {
//            plan_valid = false;
//            return;
//        }

        // Transform and store path
        current_path = path->poses;
        transformPath(current_path);
    }

    // Transform a path from map to base_link frame
    void transformPath(std::vector<geometry_msgs::PoseStamped> &path)
    {
        try
        {
            tf_listener.waitForTransform("base_link", "map", ros::Time::now(), ros::Duration(3.0));
            for (auto& path_segment : path)
            {
                tf_listener.transformPose("base_link", path_segment, path_segment);
            }
            transformed_path = current_path;
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("Error during transform: %s", ex.what());
            plan_valid = false;
        }
    }

    // Euclidean distance from two poses/ two 2dvecs
    double calcDistance(const geometry_msgs::PoseStamped &, const geometry_msgs::PoseStamped &);
    double calcDistance(const double &x_diff, const double &y_diff);

    // Angle between pose2-pose1 and x-axis
    double calcAngle(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);

    // Keep value between lower and upper bounds
    double clip(double n, double lower, double upper);

    // Sum change of angles up to a maximum distance or to the end of the path
    double calcCurveWeight(const double maxDist);

    // Simple weighting of curves. Only two points
    double simpleCurveWeight();


//
//    void getCarPose(geometry_msgs::PoseStamped &carPose) {
//        tf::StampedTransform carTransform;
//        tf_listener->lookupTransform("map", "base_link", ros::Time(0), carTransform);
//        carPose.pose.position.x = carTransform.getOrigin().getX();
//        carPose.pose.position.y = carTransform.getOrigin().getY();
//        carPose.pose.orientation.w = carTransform.getRotation().getW();
//        carPose.pose.orientation.x = carTransform.getRotation().getX();
//        carPose.pose.orientation.y = carTransform.getRotation().getY();
//        carPose.pose.orientation.z = carTransform.getRotation().getZ();
//    }
};

double SpeedController::calcSpeed()
{
//    //double curveAngle = calcCurveWeight(2.0);
    double curveWeight = simpleCurveWeight();
    std::cout << "curveWeight: " << curveWeight << std::endl;

    if (curveWeight == -1)
    {
        return 0.0;
    }


    return 0.2;
}

double SpeedController::calcDistance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
    return sqrt(pow((pose1.pose.position.x - pose2.pose.position.x), 2) +
                pow((pose1.pose.position.y - pose2.pose.position.y), 2));
}

double SpeedController::calcDistance(const double &x_diff, const double &y_diff)
{
    return sqrt(x_diff*x_diff + y_diff*y_diff);
}

double SpeedController::calcAngle(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
//    std::cout << std::setprecision(5)
//              << "y2: " << pose2.pose.position.y << " y1: " << pose1.pose.position.y << " ydiff: " << pose2.pose.position.y - pose1.pose.position.y
//              << "\nx2: " << pose2.pose.position.x << " x1: " << pose1.pose.position.x << " xdiff: " << pose2.pose.position.x - pose1.pose.position.x
//              << std::endl;
    return atan2(pose2.pose.position.y - pose1.pose.position.y, pose2.pose.position.x - pose1.pose.position.x);
}

double SpeedController::clip(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

double SpeedController::calcCurveWeight(const double maxDist)
{
    double accumulatedDistance = 0.0;
    double accumulatedAngle = 0.0;
    std::fstream logfile("/home/tas_group_04/catkin_ws/src/tas1516_car_04/speed_control/log/curve_detect.log",
                    std::ios::in | std::ios::out | std::ios::app);


    // Check if plan is up to date and contains enough segments
    if (plan_valid)
    {
        // Initialize with the first two poses so the iterative algorithm works
        double vx = 0.0, vx_prev = current_path[jump_segments].pose.position.x - current_path[0].pose.position.x;
        double vy = 0.0, vy_prev = current_path[jump_segments].pose.position.y - current_path[0].pose.position.y;
        double dist = 0.0, dist_prev = calcDistance(vx_prev, vy_prev);
        double angle = 0.0;
        // Iterate over all poses and accumulate the angle between the segments (vectors)
        // Stop when distance reaches max or end of vector
        // Leave out last 10 segments because of instabilisties <<<<< !!!!!!
        // Jump 5 segments at a time to smooth jumps <<<<<< !!!!!!
        for (int i = jump_segments; i < current_path.size()-10 && accumulatedDistance < maxDist; i += jump_segments)
        {
            // Calculate current vector translated to the origin
            vx = current_path[i].pose.position.x - current_path[i-jump_segments].pose.position.x;
            vy = current_path[i].pose.position.y - current_path[i-jump_segments].pose.position.y;

            // Calculate length of the current vector
            dist = calcDistance(vx, vy);

            // Calculate angle between current and previous vector
            // The input is bounded (clip) to avoid numerical instabilities
            angle = fabs(acos(clip((vx * vx_prev + vy * vy_prev) / (dist * dist_prev), 0.0, 1.0))*180/M_PI);

            // Logging
//            if (logfile.is_open()) {
//                logfile << "angle: " << angle
//                        << " dist: " << dist
//                        << " acc_angle: " << accumulatedAngle
//                        << " acc_dist: " << accumulatedDistance << std::endl;
//            }

            // Accumulate distance and angle
            accumulatedDistance += dist;
            accumulatedAngle += angle;

            // Save current vector for the next iteration
            dist_prev = dist;
            vx_prev = vx;
            vy_prev = vy;
        }
        logfile.close();
        //std::cout << std::setprecision(5) << "angle: " << accumulatedAngle << " distance: " << accumulatedDistance << std::endl;
    }
    else
    {
        std::cout << "No valid path" << std::endl;
    }

    return accumulatedAngle;
}

double SpeedController::simpleCurveWeight()
{
    double accumulatedDistance = 0.0;
    double shortDist = 0.4, longDist = 2.5;
    double shortAngle = 0.0, longAngle = 0.0;
    bool shortValid = false, longValid = false;
    double threshold = shortDist;
    double weight = 0.0;

    const double maxShort = 90.0, maxLong = 90.0;

    for (int i = 1; i < transformed_path.size(); i += 1)
    {
        accumulatedDistance += calcDistance(transformed_path[i], transformed_path[i-1]);
        if (accumulatedDistance > threshold)
        {
            if (threshold == longDist) // Large threshold reached - leave loop
            {
                longAngle = fabs(calcAngle(transformed_path[0], transformed_path[i]));
                longValid = true;
                std::cout << "i: " << i << " longAngle: " << longAngle*180/M_PI << " accDist: " << accumulatedDistance <<  std::endl;
                break;
            }
            else // Short threshold reached - switch to larger threshold
            {
                threshold = longDist;
                shortAngle = fabs(calcAngle(transformed_path[0], transformed_path[i]));
                shortValid = true;
                std::cout << "i: " << i  << " shortAngle: " << shortAngle*180/M_PI << " accDist: " << accumulatedDistance <<  std::endl;
            }
        }
    }

    if (shortValid)
    {
        weight += 0.6 * clip(shortAngle, 0.01, maxShort)/maxShort;
        if (longValid)
        {
            weight += 0.4 * clip(longAngle, 0.0, maxLong)/maxLong;
        }
        else
        {
            weight += 0.4;
        }
        return weight;
    }
    else
    {
        return -1.0;
    }




}

#endif // SPEED_CONTROLLER_HPP
