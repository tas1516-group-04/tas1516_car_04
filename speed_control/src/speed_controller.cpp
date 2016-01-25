#include "speed_controller.h"

// Default constructor for the SpeedController
// Adds listener to the tf topic. Initial plan is null
SpeedController::SpeedController(const tf::TransformListener *listener)
{
    transform_listener = listener;
    plan_valid = false;
}

// Callback for the NavfnROS/plan subscriber
// Transforms the given path into the base_link frame and stores it
void SpeedController::planCallback(const nav_msgs::Path::ConstPtr &path)
{
    plan_valid = false;

    // Transform and store path
    current_path = path->poses;
    // transformPath(current_path);

    plan_valid = true;
}

void SpeedController::set_parameters(ros::NodeHandle &node_handle)
{
    // Parameters from parameter server
    node_handle.param<int>("/speed_control_node/jump_segments", jump_segments, 5);
    node_handle.param<double>("/speed_control_node/angle_min", angle_min, 0.0);
    node_handle.param<double>("/speed_control_node/angle_max", angle_max, 90.0);
    node_handle.param<double>("/speed_control_node/short_limit", short_limit, 40.0);
    node_handle.param<double>("/speed_control_node/long_limit", long_limit, 40.0);
    node_handle.param<double>("/speed_control_node/short_dist", short_dist, 0.4);
    node_handle.param<double>("/speed_control_node/long_dist", long_dist, 2.0);
}


double SpeedController::calcSpeed()
{ 
    return calcCurveWeightSimple();
}

void SpeedController::transformPath(std::vector<geometry_msgs::PoseStamped> &path)
{
    try
    {
        transform_listener->waitForTransform("base_link", "map", ros::Time::now(), ros::Duration(0.3));
        for (auto& path_segment : path)
        {
            transform_listener->transformPose("base_link", path_segment, path_segment);
        }
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("Error during transform: %s", ex.what());
        plan_valid = false;
    }

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
        const int jump_segments = 10;
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
            if (logfile.is_open()) {
                logfile << "angle: " << angle
                        << " dist: " << dist
                        << " acc_angle: " << accumulatedAngle
                        << " acc_dist: " << accumulatedDistance << std::endl;
            }

            // Accumulate distance and angle
            accumulatedDistance += dist;
            accumulatedAngle += angle;

            // Save current vector for the next iteration
            dist_prev = dist;
            vx_prev = vx;
            vy_prev = vy;
        }
        logfile.close();
        ROS_INFO("\nangle: %.7lf distance: %.3lf", accumulatedAngle, accumulatedDistance);
    }
    else
    {
        ROS_INFO("No valid path");
    }

    return accumulatedAngle;
}

double SpeedController::calcCurveWeightSimple()
{
    double accumulatedDistance = 0.0;
    double shortDist = short_dist, longDist = long_dist;
    double shortAngle = 0.0, longAngle = 0.0;
    bool shortValid = false, longValid = false;
    double threshold = shortDist;
    double weight = 0.0;

    const double maxShort = short_limit, maxLong = long_limit;

    // Angle calculation with respect to the car
//    for (int i = 1; i < current_path.size(); i += 1)
//    {
//        accumulatedDistance += calcDistance(current_path[i], current_path[i-1]);
//        if (accumulatedDistance > threshold)
//        {
//            if (threshold == longDist) // Large threshold reached - leave loop
//            {
//                longAngle = fabs(calcAngle(current_path[0], current_path[i]))*180/M_PI;
//                longValid = true;
//                std::cout << " longAngle: " << longAngle << std::endl;
//                break;
//            }
//            else // Short threshold reached - switch to larger threshold
//            {
//                threshold = longDist;
//                shortAngle = fabs(calcAngle(current_path[0], current_path[i]))*180/M_PI;
//                shortValid = true;
//                std::cout << " shortAngle: " << shortAngle << std::endl;
//            }
//        }
//    }

    // Angle calculation with respect to the path
    if (current_path.size() > 0)
    {    
        double vx = 0.0, vx_base = current_path[1].pose.position.x - current_path[0].pose.position.x;
        double vy = 0.0, vy_base = current_path[1].pose.position.y - current_path[0].pose.position.y;
        double dist = 0.0, dist_base = calcDistance(vx_base, vy_base);
    
    for (int i = 1; i < current_path.size(); i += 1)
    {
        accumulatedDistance += calcDistance(current_path[i], current_path[i-1]);
        if (accumulatedDistance > threshold)
        {
            if (threshold == longDist) // Large threshold reached - leave loop
            {
                vx = current_path[i].pose.position.x - current_path[0].pose.position.x;
                vy = current_path[i].pose.position.y - current_path[0].pose.position.y;
                dist = calcDistance(vx, vy);

                longAngle = fabs(acos(clip((vx * vx_base + vy * vy_base) / (dist * dist_base), 0.0, 1.0))*180/M_PI);
                longValid = true;

                // std::cout << " longAngle: " << longAngle << std::endl;
                break;
            }
            else // Short threshold reached - switch to larger threshold
            {
                vx = current_path[i].pose.position.x - current_path[0].pose.position.x;
                vy = current_path[i].pose.position.y - current_path[0].pose.position.y;
                dist = calcDistance(vx, vy);
                threshold = longDist;

                shortAngle = fabs(acos(clip((vx * vx_base + vy * vy_base) / (dist * dist_base), 0.0, 1.0))*180/M_PI);
                shortValid = true;

                // std::cout << " shortAngle: " << shortAngle << std::endl;
            } 
        }   
    }
    accumulatedDistance = 0;
    threshold = 0.1;
    double angle = 0.0;
    for (int i = 1; i < current_path.size(); i+= 1)
    {
        accumulatedDistance += calcDistance(current_path[i], current_path[i-1]);
        if (accumulatedDistance > threshold)
        {
            vx = current_path[i].pose.position.x - current_path[0].pose.position.x;
            vy = current_path[i].pose.position.y - current_path[0].pose.position.y;
            dist = calcDistance(vx, vy);

            angle = fabs(acos(clip((vx * vx_base + vy * vy_base) / (dist * dist_base), 0.0, 1.0))*180/M_PI);
            threshold += 0.1;
            std::cout << threshold << "," << angle << ",";
        }
    }
    std::cout << std::endl;
    }
    // Convert angles to curve weights
    if (shortValid)
    {
        weight += 0.5 * clip(shortAngle, 0.01, maxShort)/maxShort;
        if (longValid)
        {
            weight += 0.5 * clip(longAngle, 0.01, maxLong)/maxLong;
        }
        else
        {
            weight += weight;
        }
        std::cout << "weight: " << weight << std::endl;
        return weight;
    }
    else
    {
        return -1.0;
    }


}



