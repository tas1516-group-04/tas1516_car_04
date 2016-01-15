#include <pluginlib/class_list_macros.h>
#include "tas_local_planner.h"
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(tas_local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

using namespace std;

//Default Constructor
namespace tas_local_planner {

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    laser_geometry::LaserProjection projector_;

    if(!useBaseLinkFrame_) {
        if(!tf_->waitForTransform(
                    scan->header.frame_id,
                    "/laser",
                    scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                    ros::Duration(1.0))){
            return;
        }
        projector_.transformLaserScanToPointCloud("/laser",*scan, tlpLaserCloud,*tf_);
    } else {
        // wait for transformation
        if(!tf_->waitForTransform(
                    scan->header.frame_id,
                    "/base_link",
                    scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                    ros::Duration(1.0))){
            return;
        }

        // transform laser data to point cloud(base_link)
        projector_.transformLaserScanToPointCloud("/base_link",*scan, tlpLaserCloud,*tf_);
    }
}

LocalPlanner::LocalPlanner (){
}

LocalPlanner::LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, tf, costmap_ros);
}

void LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_) {
        subScan_ = nodeHandle_.subscribe("scan", 1000, scanCallback);
        pubTest_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("test",1000);
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        goalIsReached_ = false;

        //parameters
        nodeHandle_.param<double>("/move_base_node/car_width", carwidth_, 0.7);
        nodeHandle_.param<double>("/move_base_node/wheelbase", wheelbase_, 0.3);
        nodeHandle_.param<bool>("/move_base_node/obstacle_avoidance", doObstacleAvoidance_, false);
        nodeHandle_.param<double>("/move_base_node/min_distance", minDistance_, 0.3);
        nodeHandle_.param<double>("/move_base_node/steering_angle_parameter", steeringAngleParameter_, 0.6);
        nodeHandle_.param<double>("/move_base_node/laser_max_dist", laserMaxDist_, 2);
        nodeHandle_.param<bool>("/move_base_node/use_base_link_frame", useBaseLinkFrame_, false);
        nodeHandle_.param<int>("/move_base_node/min_object_size", minObjectSize_, 1);

        //output parameter
        if(doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance active!");
        if(!doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance inactive!");
        if(useBaseLinkFrame_) ROS_INFO("TLP: Using base_link frame!");
        if(!useBaseLinkFrame_) ROS_INFO("TLP: Using laser frame!");

        //classes
        objectAvoidance = new ObjectAvoidance(wheelbase_, carwidth_);

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
            if(!useBaseLinkFrame_) {
                tf_->waitForTransform("/laser", "/map", it->header.stamp, ros::Duration(3.0));
                tf_->transformPose("/laser", *it, *it);
            } else {
                tf_->waitForTransform("/base_link", "/map", it->header.stamp, ros::Duration(3.0));
                tf_->transformPose("/base_link", *it, *it);
            }
        }

        /// get target point
        int point = 1; // which point first? distance?
        while(calcDistance(plan_[0], plan_[point]) < minDistance_) {
            point++;
            if(point == plan_.size() - 1) break;
        }

        // should angle decrease over distance?
        while(abs(atan2(0-plan_[point].pose.position.x, 0-plan_[point].pose.position.y) + M_PI/2) < 0.3){
            point++;
            if(point == plan_.size() - 1) break;
        }

        if(oldPoint != point) ROS_INFO("TLP: Target Point: %i", point);
        oldPoint = point;

        // calc simple steering angle
        float steeringAngle = calcAngle(plan_[point].pose.position.x,plan_[point].pose.position.y);

        /// obstacle avoidance
        if(doObstacleAvoidance_) {
//            cmd_vel.angular.z = objectAvoidance->doObstacleAvoidance(steeringAngle, tlpLaserCloud)*
//                    steeringAngleParameter_*(minDistance_/calcDistance(plan_[0], plan_[point]));
            cmd_vel.angular.z = steeringAngle*steeringAngleParameter_;
        } else {
            // steering parameters decreases over distance
//            cmd_vel.angular.z = steeringAngle *
//                    steeringAngleParameter_*(minDistance_/calcDistance(plan_[0], plan_[point]));
            cmd_vel.angular.z = steeringAngle*steeringAngleParameter_;
        }
        cmd_vel.linear.x = 0.2;
    }
    steerAngleOld_ = cmd_vel.angular.z;
    return true;
}
bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    globalPlanIsSet_ = true;
    if(plan_.size() < 50) {
        goalIsReached_ = true;
    } else {
        goalIsReached_ = false;
    }
    plan_ = plan;
}
bool LocalPlanner::isGoalReached() {
    return goalIsReached_;
}

void LocalPlanner::analyzeLaserData(float angle)
{
    //transform to vector form
    float r = abs(wheelbase_/tan(angle));
    laserDataTf_.clear();
    int numberLaserPoints = (int) ( (abs(tlpLaserScan->angle_min) + abs(tlpLaserScan->angle_max))/tlpLaserScan->angle_increment);
    int startLaserPoint = 50;
    int endLaserPoint   = numberLaserPoints-50;
    for(int i = startLaserPoint; i < endLaserPoint; i++) {
        //max distance
        if(tlpLaserScan->ranges[i] < laserMaxDist_) {
            geometry_msgs::Pose newLaserPoint;
            newLaserPoint.position.x = cos(tlpLaserScan->angle_min + tlpLaserScan->angle_increment*i)*tlpLaserScan->ranges[i];
            newLaserPoint.position.y = sin(tlpLaserScan->angle_min + tlpLaserScan->angle_increment*i)*tlpLaserScan->ranges[i];
            laserDataTf_.push_back(newLaserPoint);
        }
    }
}

// distance between two PoseStamped poses
float LocalPlanner::calcDistance(geometry_msgs::PoseStamped& a, geometry_msgs::PoseStamped& b) {
    float xDiff = a.pose.position.x - b.pose.position.x;
    float yDiff = a.pose.position.y - b.pose.position.y;
    return sqrt(pow(xDiff,2) + pow(yDiff,2));
}

float LocalPlanner::calcAngle(float x, float y) {
    // radius from wheelbase and steerAngle
    if(y == 0) return 0;
    float radius = pow(x,2)+pow(y,2)/(2*y);
    return atan(wheelbase_/radius);
}
};
