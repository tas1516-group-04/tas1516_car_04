#include <pluginlib/class_list_macros.h>
#include "tas_local_planner.h"
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(tas_local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

using namespace std;

//Default Constructor
namespace tas_local_planner {

laser_geometry::LaserProjection projector_;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
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
        if(doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance active!");
        if(!doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance inactive!");
        if(useBaseLinkFrame_) ROS_INFO("TLP: Using base_link frame!");
        if(!useBaseLinkFrame_) ROS_INFO("TLP: Using laser frame!");

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
        // costmap Global Frame ID = odom

        // transformation from map to laser frame
        for(std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin(); it != plan_.end(); it++) {
            if(!useBaseLinkFrame_) {
                tf_->waitForTransform("/laser", "/map", it->header.stamp, ros::Duration(3.0));
                tf_->transformPose("/laser", *it, *it);
            } else {
                tf_->waitForTransform("/base_link", "/map", it->header.stamp, ros::Duration(3.0));
                tf_->transformPose("/base_link", *it, *it);
            }
        }
        // plan_ now in base_link frame

        /// calc steerAngle from trajectorie
        // TODO: which point from plan? depending on distance?
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

        ROS_INFO("TLP: Target Point: %i", point);

        // calc simple steering angle
        float steerAngle = calcAngle(plan_[point].pose.position.x,plan_[point].pose.position.y)*
                steeringAngleParameter_*(minDistance_/calcDistance(plan_[0], plan_[point]));

        /// obstacle detection
        int objectSize = 0;

        pubTest_.publish(tlpLaserCloud);
        if(!useBaseLinkFrame_) {
            //            analyzeLaserData(steerAngle);
            for(std::vector<geometry_msgs::Point32>::iterator it = tlpLaserCloud.points.begin(); it != tlpLaserCloud.points.end(); it++){
                if(checkForObject(steerAngleOld_, it->x, it->y)) {
                    objectSize++;
                } else {
                    objectSize = 0;
                }

                // break if object is big enough
                if(objectSize > minObjectSize_) break;
            }
        } else {
            for(std::vector<geometry_msgs::Point32>::iterator it = tlpLaserCloud.points.begin(); it != tlpLaserCloud.points.end(); it++){
                if(checkForObject(steerAngleOld_, it->x, it->y)) {
                    objectSize++;
                } else {
                    objectSize = 0;
                }

                // break if object is big enough
                if(objectSize > minObjectSize_) break;
            }
        }

        /// obstacle avoidance
        // search for avoidance path
        if(objectSize > 0 && doObstacleAvoidance_) {
            int helper = 1;
            //steerAngle = 0; //for testing
            while(true) {
                // bool objectLeft = true;
                // bool objectRight = true;
                int objectLeftSize = 0;
                int objectRightSize = 0;

                //                float angleInc =  steerAngle + helper*0.05;
                //                float angleDec =  steerAngle - helper*0.05;
                float angleInc = steerAngleOld_ + helper * 0.05;
                float angleDec = steerAngleOld_ - helper * 0.05;

                if(!useBaseLinkFrame_) {
                    for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it != laserDataTf_.end(); it++){
                        if(checkForObject(angleInc, it->position.x, it->position.y)) objectLeftSize++;
                        if(checkForObject(angleDec, it->position.x, it->position.y)) objectRightSize++;
                        if(objectLeftSize > 5 || objectRightSize > 5) break;
                    }
                } else {
                    for(std::vector<geometry_msgs::Point32>::iterator it = tlpLaserCloud.points.begin(); it != tlpLaserCloud.points.end(); it++){
                        if(checkForObject(angleInc, it->x, it->y)) objectLeftSize++;
                        if(checkForObject(angleDec, it->x, it->y)) objectRightSize++;
                        if(objectLeftSize > 5 || objectRightSize > 5) break;
                    }
                }

                // decide what to do
                if(objectLeftSize < 6) {
                    cmd_vel.angular.z = angleInc; // which steering parameter?
                    ROS_INFO("TLP: Alternative: Left turn! Z: %f", (float) cmd_vel.angular.z);
                    break;
                }
                if(objectRightSize < 6) {
                    cmd_vel.angular.z = angleDec;
                    ROS_INFO("TLP: Alternative: Right turn! Z: %f",(float) cmd_vel.angular.z);
                    break;
                }
                if(helper == 10) {
                    ROS_INFO("TLP: No alternative found!");
                    break;
                }
                helper++;
            }
            cmd_vel.linear.x = 0.2;
            //cmd_vel.angular.z = steerAngle*0.6; //remove!
        } else {
            // steering parameters decreases over distance
            cmd_vel.angular.z = steerAngle;
            cmd_vel.linear.x = 0.2;
        }
    }
    steerAngleOld_ = cmd_vel.angular.z;
    return true;
}
bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    //ROS_INFO("TLP: new global plan received! length: %i", (int) plan.size());
    globalPlanIsSet_ = true;
    //    if(calcDistance(plan_.front(), plan_.back()) < minDistance_) {
    //        goalIsReached_ = true;
    //    } else {
    //        goalIsReached_ = false;
    //    }
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

    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "laser";
    poseArray.poses = laserDataTf_;
    //ROS_INFO("TLP: publish laserDataTf_ with size: %i", (int) poseArray.poses.size());
    //ROS_INFO("TLP: %i laser objects detected!", (int) laserObjects_.size());
    //pubTest_.publish(poseArray);
}

void LocalPlanner::tfRobotPose()
{
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

bool LocalPlanner::checkForObject(float angle, float x, float y) {
    float r = wheelbase_/tan(angle);
    if(r > 0) {
        // x^2 +(y-r-w/2)^2-r^2
        if(pow(x,2) + pow(y-r,2) - pow(r-carwidth_/2,2) >= 0 && pow(x,2) + pow(y-r,2) -pow(r+carwidth_/2,2) <= 0) {
            // return true if object in path
            if(sqrt(pow(x,2)+pow(y,2)) < 1) {
                return true;
            } else {
                return false;
            }
        } else {
            // return false otherwise
            return false;
        }
    } else {
        if(pow(x,2) + pow(y-r,2) - pow(r+carwidth_/2,2) >= 0 && pow(x,2) + pow(y-r,2) - pow(r-carwidth_/2,2) <= 0) {
            // return true if object in path
            if(sqrt(pow(x,2)+pow(y,2)) < 1) {
                return true;
            } else {
                return false;
            }
        } else {
            // return false otherwise
            return false;
        }
    }
}

};
