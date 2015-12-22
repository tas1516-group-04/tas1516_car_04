#include "speed_controller.h"

// Default constructor for the SpeedController
// Adds listener to the tf topic. Initial plan is null
SpeedController::SpeedController(ros::NodeHandle &nh, const tf::TransformListener *listener) :
    node_handle(nh),
    transform_listener(listener)
{
    plan_valid = false;
    cmd_velocity = 0.0;

    cmd_subscriber = node_handle.subscribe<geometry_msgs::Twist>("cmd_vel2", 1000, &SpeedController::cmdCallback, this);
    cmd_publisher = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    node_handle.param<int>("/speed_control_node/jump_segments", jump_segments, 5);
    node_handle.param<double>("/speed_control_node/angle_min", angle_min, 0.0);
    node_handle.param<double>("/speed_control_node/angle_max", angle_max, 90.0);
}

// Callback for the NavfnROS/plan subscriber
// Transforms the given path into the base_link frame and stores it
void SpeedController::planCallback(const nav_msgs::Path::ConstPtr &path)
{
    // Assume valid plan
    plan_valid = true;
    // Need at least some elements in path
    if (path->poses.size() < jump_segments+15)
    {
        plan_valid = false;
        return;
    }

    // Transform and store path
    current_path = path->poses;
    transformPath(current_path);
}

void SpeedController::calcSpeed()
{
    double curveAngle = calcCurveWeight(2.0);
    curveAngle = clip(curveAngle, angle_min, angle_max);
    if (plan_valid)
    {
    	cmd_velocity = 1 - (curveAngle - angle_min) / angle_max;
    }
    else
    {
	cmd_velocity = 0;
    }
    std::cout << "velocity: " << cmd_velocity << " angle: " << curveAngle << std::endl;
}

void SpeedController::cmdCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel2)
{
    // Load velocity command from the speed controller
    geometry_msgs::Twist cmd_vel = *cmd_vel2;
    cmd_vel.linear.x = cmd_velocity;
    cmd_publisher.publish(cmd_vel);
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
        std::cout << std::setprecision(5) << "angle: " << accumulatedAngle << " distance: " << accumulatedDistance << std::endl;
    }
    else
    {
        std::cout << "No valid path" << std::endl;
    }

    return accumulatedAngle;
}



