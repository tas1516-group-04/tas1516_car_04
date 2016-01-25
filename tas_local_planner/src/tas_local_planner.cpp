#include <pluginlib/class_list_macros.h>
#include "tas_local_planner.h"
#include <tf/transform_datatypes.h>

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
        costmap_ros_ = costmap_ros;
        goalIsReached_ = false;

        //parameters
        nodeHandle_.param<double>("/move_base_node/car_width", carwidth_, 0.3);
        nodeHandle_.param<double>("/move_base_node/wheelbase", wheelbase_, 0.3);
        nodeHandle_.param<bool>("/move_base_node/obstacle_avoidance", doObstacleAvoidance_, false);
        nodeHandle_.param<double>("/move_base_node/min_distance", minDistance_, 0.3);
        nodeHandle_.param<double>("/move_base_node/corridor_width", corridorWidth_, 0.4);
        nodeHandle_.param<double>("/move_base_node/steering_angle_parameter", steeringAngleParameter_, 0.6);
        nodeHandle_.param<double>("/move_base_node/laser_max_dist", laserMaxDist_, 2);
        nodeHandle_.param<double>("/move_base_node/offset", offset_, 2);
        nodeHandle_.param<double>("/move_base_node/min_object_size", minObjectSize_, 1);

        //classes
        subScan_ = nodeHandle_.subscribe("scan", 1000, &LocalPlanner::scanCallback, this);
        //pubScanTf_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("scanTf", 1000);

        //output parameter
        if(doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance active!");
        if(!doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance inactive!");

        //finish initialization
        ROS_INFO("TAS LocalPlanner succesfully initialized!");
        initialized_ = true;
    } else {
        ROS_INFO("TAS LocalPlanner is already initialized!");
    }
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

    //calc angular component of cmd_vel
    if(globalPlanIsSet_ && goalIsReached_ == false) {
        /// transform global plan
        for(std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin(); it != plan_.end(); it++) {
            tf_->waitForTransform("/laser", "/map", it->header.stamp, ros::Duration(3.0));
            tf_->transformPose("/laser", *it, *it);
        }

        /// get target point
        int point = 0; // which point first? distance?
        geometry_msgs::PoseStamped origin;
        origin.pose.position.x = 0;
        origin.pose.position.y = 0;

        /* old
        while(calcDistance(origin, plan_[point]) < minDistance_ || plan_[point].pose.position.x < 0) {
           point++;
            if(point == plan_.size() - 1) break;
        }

        // should angle decrease over distance?

        while(abs(atan2(0-plan_[point].pose.position.x, 0-plan_[point].pose.position.y) + M_PI/2) < 0.3){
            point++;
            if(point == plan_.size() - 1) break;
        }
        */
        //
        while(calcDistance(origin, plan_[point]) < minDistance_ || plan_[point].pose.position.x < 0) {
            point++;
            if(point == plan_.size() - 1) break;
        }

        // max angle decreases over distance: angle = atan(carwidth_/distance)
        if(!(point == plan_.size() - 1)) {
            while(abs(plan_[point].pose.position.y) < corridorWidth_){
                point++;
                if(point == plan_.size() - 1) break;
            }
        }

        // calc simple steering angle
        double steeringAngle;
        if(doObstacleAvoidance_) {
            steeringAngle = calcAngle(doObstacleAvoidance(point, plan_, cmd_vel));
        } else {
            cmd_vel.linear.y = 1.0;
            steeringAngle = calcAngle(plan_[point]);
            if(oldPoint != point) {
                ROS_INFO("TLP: T.P. %i | x: %f | y: %f | a: %f",
                         point,
                         plan_[point].pose.position.x,
                         plan_[point].pose.position.y,
                         steeringAngle);
            }
            oldPoint = point;
        }

        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = steeringAngle*steeringAngleParameter_ + offset_;
        //cmd_vel.angular.z = steeringAngle*atan(steeringAngle/cmd_vel.linear.x*wheelbase_) + offset_;
    }
    return true;
}

bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    globalPlanIsSet_ = true;
    if(plan.size() < 50) {
        goalIsReached_ = true;
    } else {
        goalIsReached_ = false;
    }
    plan_ = plan;
}
bool LocalPlanner::isGoalReached() {
    return goalIsReached_;
}

void LocalPlanner::filterLaserScan()
{
    laserPoints.clear();
    int numberLaserPoints = (int) ( (abs(scan_->angle_min) + abs(scan_->angle_max))/scan_->angle_increment);
    for(int i = 0; i < numberLaserPoints-1; i++) {
        geometry_msgs::Point32 newLaserPoint;
        newLaserPoint.x = cos(scan_->angle_min + scan_->angle_increment*i)*scan_->ranges[i];
        newLaserPoint.y = sin(scan_->angle_min + scan_->angle_increment*i)*scan_->ranges[i];
        if(abs(newLaserPoint.y) < corridorWidth_ + wheelbase_/2
                || newLaserPoint.x < minDistance_) {
            laserPoints.push_back(newLaserPoint);
        }
    }
    /*
    sensor_msgs::PointCloud pcl;
    pcl.header.frame_id = "laser";
    pcl.points = laserPoints;
    pubScanTf_.publish(pcl);
    */
    //ROS_INFO("TLP: %i laser points", (int) laserPoints.size());
}

bool LocalPlanner::objectInPath(geometry_msgs::PoseStamped targetPoint)
{
    int consecutivePointsInPath = 0;
    double maxConsecutivePointsInPath =  minObjectSize_/(tan(2.8/640)*distToPoint_);
    for(std::vector<geometry_msgs::Point32>::iterator it = laserPoints.begin(); it != laserPoints.end(); it++){
        // point has to be in path and in range
        if(pointInPath(it->x, it->y, targetPoint)) {
            consecutivePointsInPath++;
        } else {
            if(consecutivePointsInPath > 1+maxConsecutivePointsInPath) return true;
            consecutivePointsInPath = 0;
        }
    }
    return false;
}

bool LocalPlanner::pointInPath(double x, double y, geometry_msgs::PoseStamped targetPoint)
{
    if(pow(x-targetPoint.pose.position.x,2) + pow(y-targetPoint.pose.position.y,2) <= pow(carwidth_/2+0.1,2)){
        // return true if point in path
        return true;
    } else {
        // return false otherwise
        return false;
    }
}

geometry_msgs::PoseStamped LocalPlanner::getNewTargetPoint(geometry_msgs::PoseStamped targetPoint)
{
    geometry_msgs::PoseStamped yInc;
    yInc.pose.position.x = targetPoint.pose.position.x;
    geometry_msgs::PoseStamped yDec;
    yDec.pose.position.x = targetPoint.pose.position.x;
    for(int i = 1; i < 20; i++) {
        yInc.pose.position.y = targetPoint.pose.position.y + i * 0.05;
        yDec.pose.position.y = targetPoint.pose.position.y - i * 0.05;

        // decide what to do
        if(!objectInPath(yInc)) {
            ROS_INFO("TLP: Alternative: Left turn! ");
            ROS_INFO("TLP: o.a. | x: %f | y: %f ",
                     yInc.pose.position.x,
                     yInc.pose.position.y);
            return yInc; // which steering parameter?
        }
        if(!objectInPath(yDec)) {
            ROS_INFO("TLP: Alternative: Right turn!");
            ROS_INFO("TLP: o.a. | x: %f | y: %f ",
                     yDec.pose.position.x,
                     yDec.pose.position.y);
            return yDec;
        }
    }
    return targetPoint;
}

geometry_msgs::PoseStamped LocalPlanner::doObstacleAvoidance(int targetPoint, std::vector<geometry_msgs::PoseStamped> plan, geometry_msgs::Twist &cmd_vel)
{
    filterLaserScan();
    for(std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin(); it != plan.begin()+targetPoint; it++) {
        distToPoint_ = sqrt(pow(it->pose.position.x,2)+pow(it->pose.position.y,2));
        if(objectInPath(*it)) {
            ROS_INFO("TLP: Object in Path!");
            // break if object in path
            cmd_vel.linear.y = 0.0;
            return getNewTargetPoint(*it);
        }
        // safety mechanism
        //if(it == plan.end()) break;
    }
    // dont break if no object in path
    ROS_INFO("TLP: T.P. %i | x: %f | y: %f",
             targetPoint,
             plan[targetPoint].pose.position.x,
             plan[targetPoint].pose.position.y);
    cmd_vel.linear.y = 1.0;
    return plan[targetPoint];
}

void LocalPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    scan_ = scan;
}

// distance between two PoseStamped poses
float LocalPlanner::calcDistance(geometry_msgs::PoseStamped& a, geometry_msgs::PoseStamped& b) {
    float xDiff = a.pose.position.x - b.pose.position.x;
    float yDiff = a.pose.position.y - b.pose.position.y;
    return sqrt(pow(xDiff,2) + pow(yDiff,2));
}

double LocalPlanner::calcAngle(geometry_msgs::PoseStamped point) {
    // radius from wheelbase and steerAngle
    if(point.pose.position.y == 0) return 0;

    // calc circle center
    double xM = (-1)*wheelbase_;
    double yM = (pow(point.pose.position.x+wheelbase_,2)+pow(point.pose.position.y,2) - pow(wheelbase_,2))/(2*point.pose.position.y);

    // calc radius
    double radius = sqrt(pow(xM,2) + pow(yM,2));

    // calc steering angle
    double angle = M_PI/2 - acos(wheelbase_/radius);
    if(yM < 0) angle = angle*(-1);
    return angle;
}
};
