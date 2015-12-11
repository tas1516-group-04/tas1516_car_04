#include <pluginlib/class_list_macros.h>
#include "tas_local_planner.h"
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(tas_local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

using namespace std;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    tlpLaserScan = scan;
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

    //transform robot pose
    //ROS_INFO("TLP: computeVelocityCommands!");

    /*
    tf::Stamped<tf::Pose> tempRobotPose;
    costmap_ros_->getRobotPose(tempRobotPose);
    tf::poseStampedTFToMsg(tempRobotPose, robotPose_);
    //--

    //calc which path point is closest to current robot pose
    float lowestDist;
    int counter = 1;
    int nearestPathPoint = 0;
    lowestDist = calcDistance(robotPose_, *(plan_.begin()));
    for(std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin()+1; it != plan_.end(); ++it) {
        if(calcDistance(robotPose_, *it) < lowestDist) {
            nearestPathPoint = counter;
            lowestDist = calcDistance(robotPose_, *it);
        }
        counter ++;
    }
    //--
    ROS_INFO("Nearest path point: %i", nearestPathPoint);
    ROS_INFO("Distance: %f", lowestDist);
    */

    analyzeLaserData();

    //ROS_INFO("Range 320: %f", tlpLaserScan->ranges[320]);

    // emergency stop
    bool stopCar = false;
    for(int i = 0; i < 620; i++) { //620? passt des?
        if(tlpLaserScan->ranges[i] < 0.1) {
            stopCar = true;
        }
    }
    if(stopCar) {
        cmd_vel.linear.x = 0;
        ROS_INFO("TLP: WARNING! Obstacle in front!");
        return true;
    } else {
        cmd_vel.linear.x = 0.5;
        return true;
    }
}
bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    int gbpLength = plan.size();
    ROS_INFO("TLP: new global plan received! length: %i", gbpLength);
    plan_ = plan;
    return true;
}
bool LocalPlanner::isGoalReached() {
    return goalIsReached_;
}

void LocalPlanner::analyzeLaserData()
{
    //transform to vector form
    std::vector<geometry_msgs::Point> tmpLaserData;
    for(int i = 0; i < 620; i++) { //620? passt des?
        geometry_msgs::Point laserPoint;
        laserPoint.x = sin(-1.4 + (2.8*i)/620)*tlpLaserScan->ranges[i];
        laserPoint.y = cos(-1.4 + (2.8*i)/620)*tlpLaserScan->ranges[i];
        tmpLaserData.push_back(laserPoint);
    }
    //calc direction
    std::vector<geometry_msgs::Point> tmpLaserDirection;
    for(std::vector<geometry_msgs::Point>::iterator it = tmpLaserData.begin()+1; it!=tmpLaserData.end(); it++) {
        geometry_msgs::Point laserPointDirection;
        laserPointDirection.x = it->x - (it-1)->x;
        laserPointDirection.y = it->y - (it-1)->y;
        tmpLaserDirection.push_back(laserPointDirection);
    }

    //retrieve objects
    laserObjects_.clear();
    LaserObject newObject;
    newObject.start = 1;
    int helper = 1;
    for(std::vector<geometry_msgs::Point>::iterator it = tmpLaserDirection.begin()+1; it!=tmpLaserDirection.end(); it++) {
        //if angle between two directions is greater than ? start new Object
        if(abs(atan2(it->x, it->y) - atan2((it-1)->x,(it-1)->y)) < 1 || it == tmpLaserDirection.end()) {
            newObject.end = helper;
        } else {
            if(newObject.end - newObject.start > 10) laserObjects_.push_back(newObject);
            newObject.start = helper;
        }
        //float angle = atan2(it->x, it->y) - atan2((it-1)->x,(it-1)->y);
        //ROS_INFO("TLP: angle %i: %f", helper, angle);
        helper ++;
    }

    ROS_INFO("TLP: %i laser objects detected!", (int) laserObjects_.size());
}

};
