#include <pluginlib/class_list_macros.h>
#include "tas_local_planner.h"
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(tas_local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

using namespace std;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    tlpLaserScan = scan;
    // ROS_INFO("Min angle: %f", (float) scan->angle_min);
    // ROS_INFO("Max angle: %f", (float) scan->angle_max);
    // ROS_INFO("Angle increment: %f", (float) scan->angle_increment);
}

float calcDistance(geometry_msgs::PoseStamped& a, geometry_msgs::PoseStamped& b) {
    float xDiff = a.pose.position.x - b.pose.position.x;
    float yDiff = a.pose.position.y - b.pose.position.y;
    return sqrt(pow(xDiff,2) + pow(yDiff,2));
}

//Default Constructor
namespace tas_local_planner {

LocalPlanner::LocalPlanner (){
}

LocalPlanner::LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, tf, costmap_ros);
}

void LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_) {
        subScan_ = nodeHandle_.subscribe("scan", 1000, scanCallback);
        pubTest_ = nodeHandle_.advertise<geometry_msgs::PoseArray>("test",1000);
        tf_ = tf;
        costmap_ros_ = costmap_ros_;
        goalIsReached_ = false;

        //finish initialization
        ROS_INFO("TAS LocalPlanner succesfully initialized!");
        initialized_ = true;
    } else {
        ROS_INFO("TAS LocalPlanner is already initialized!");
    }
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

    analyzeLaserData();

    //calc angular component of cmd_vel
    float angleParameter = 1;
    if(globalPlanIsSet_) {
        tfRobotPose(); //to transfrom robot pose !
        geometry_msgs::Vector3 angularVec;
        float angularFloat = (2*asin(robotPose_.pose.orientation.w) - 2*asin(plan_[0].pose.orientation.w)) * (angleParameter);
        angularVec.x = cos(angularFloat/2);
        angularVec.y = sin(angularFloat/2);
        ROS_INFO("Costmap Global Frame: %s", costmap_ros_->getGlobalFrameID().c_str());
    }

    // emergency stop
    bool stopCar = false;
    for(int i = 0; i < 640; i++) { //620? passt des?
        if(tlpLaserScan->ranges[i] < 0.5) {
            stopCar = true;
        }
    }
    if(stopCar) {
        cmd_vel.linear.x = 0;
        ROS_INFO("TLP: WARNING! Obstacle in front!");
        return true;
    } else {
        cmd_vel.linear.x = 0.2;
        return true;
    }
}
bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    ROS_INFO("TLP: new global plan received! length: %i", (int) plan.size());
    globalPlanIsSet_ = true;
    plan_ = plan;
}
bool LocalPlanner::isGoalReached() {
    return goalIsReached_;
}

void LocalPlanner::analyzeLaserData()
{
    //transform to vector form
    laserDataTf_.clear();
    int numberLaserPoints = (int) ( (abs(tlpLaserScan->angle_min) + abs(tlpLaserScan->angle_max))/tlpLaserScan->angle_increment);
    for(int i = 0; i < numberLaserPoints; i++) {
        //max distance
        if(tlpLaserScan->ranges[i] < 100) {
            geometry_msgs::Pose newLaserPoint;
            newLaserPoint.position.x = cos(tlpLaserScan->angle_min + tlpLaserScan->angle_increment*i)*tlpLaserScan->ranges[i];
            newLaserPoint.position.y = sin(tlpLaserScan->angle_min + tlpLaserScan->angle_increment*i)*tlpLaserScan->ranges[i];
            laserDataTf_.push_back(newLaserPoint);
        }
    }
    //calc direction
    for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it!=laserDataTf_.end()-1; it++) {
        float theta = atan2((it+1)->position.x-it->position.x, (it+1)->position.y-it->position.y) - M_PI/2;
        it->orientation.w = sin(theta/2);
        it->orientation.z = cos(theta/2);
    }

    laserObjects_.clear();
    LaserObject newObject;
    int helper = 0;
    bool objectStarted = false;
    for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it!=laserDataTf_.end()-1; it++) {
        if(abs(acos(it->orientation.z) - acos((it+1)->orientation.z)) < 0.5) {
            if(!objectStarted) {
                newObject.start = helper;
                objectStarted = true;
            } else {
                newObject.end = helper;
            }
        } else {
            if(objectStarted && newObject.end - newObject.start > 10) {
                laserObjects_.push_back(newObject);
                objectStarted = false;
            } else {
                objectStarted = false;
            }
        }
    }

    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "laser";
    poseArray.poses = laserDataTf_;
    //ROS_INFO("TLP: publish laserDataTf_ with size: %i", (int) poseArray.poses.size());
    //ROS_INFO("TLP: %i laser objects detected!", (int) laserObjects_.size());
    pubTest_.publish(poseArray);
}

void LocalPlanner::tfRobotPose()
{
    //transform RobotPose
    tf::Stamped<tf::Pose> tempRobotPose;
    costmap_ros_->getRobotPose(tempRobotPose);
    tf::poseStampedTFToMsg(tempRobotPose, robotPose_);
}

};
