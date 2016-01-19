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
        nodeHandle_.param<double>("/move_base_node/steering_angle_parameter", steeringAngleParameter_, 0.6);
        nodeHandle_.param<double>("/move_base_node/laser_max_dist", laserMaxDist_, 2);
        nodeHandle_.param<double>("/move_base_node/offset", offset_, 2);

        //classes
        objectAvoidance = new ObjectAvoidance(wheelbase_, carwidth_, tf);
        nodeHandle_.param<int>("/move_base_node/min_object_size", objectAvoidance->minObjectSize_, 1);

        subScan_ = nodeHandle_.subscribe("scan", 1000, &ObjectAvoidance::scanCallback, objectAvoidance);

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
        int point = 1; // which point first? distance?
        geometry_msgs::PoseStamped origin;
        origin.pose.position.x = 0;
        origin.pose.position.y = 0;
        while(calcDistance(origin, plan_[point]) < minDistance_ || plan_[point].pose.position.x < 0) {
           point++;
            if(point == plan_.size() - 1) break;
        }

        // should angle decrease over distance?
        while(abs(atan2(0-plan_[point].pose.position.x, 0-plan_[point].pose.position.y) + M_PI/2) < 0.3){
            point++;
            if(point == plan_.size() - 1) break;
        }

        // calc simple steering angle
        double steeringAngle;
        if(doObstacleAvoidance_) {
            steeringAngle = calcAngle(objectAvoidance->doObstacleAvoidance(point, plan_));
        } else {
            steeringAngle = calcAngle(plan_[point]);
        }

        if(oldPoint != point) {
            ROS_INFO("TLP: T.P. %i | x: %f | y: %f | a: %f",
                     point,
                     plan_[point].pose.position.x,
                     plan_[point].pose.position.y,
                     steeringAngle);
        }
        oldPoint = point;

        cmd_vel.angular.z = steeringAngle*steeringAngleParameter_ + offset_;
        cmd_vel.linear.x = 0.5;
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

    // give object avoidance information
    objectAvoidance->radius = radius;
    objectAvoidance->yM = yM;

    // calc steering angle
    double angle = M_PI/2 - acos(wheelbase_/radius);
    if(yM < 0) angle = angle*(-1);
    return angle;
}
};
