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

// distance between two PoseStamped poses
float calcDistance(geometry_msgs::PoseStamped& a, geometry_msgs::PoseStamped& b) {
    float xDiff = a.pose.position.x - b.pose.position.x;
    float yDiff = a.pose.position.y - b.pose.position.y;
    return sqrt(pow(xDiff,2) + pow(yDiff,2));
}

float calcAngle(float x, float y, float &radiusOut) {
    // radius from wheelbase and steerAngle
    //float radius = WHEELBASE / tan(steerAngle);
    if(y == 0) return 0;
    radiusOut = abs((pow(x,2)+pow(y,2))/(2*y));
    return atan(WHEELBASE/radiusOut);
}

bool checkForObject(float r, float x, float y) {
    if(pow(x,2) + pow(y-r-CARWIDTH/2,2) - pow(r,2) > 0 && pow(x,2) + pow(y-r+CARWIDTH/2,2) < 0) {
        // return true if object in path
        return true;
    } else {
        // return false otherwise
        return false;
    }
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
        costmap_ros_ = costmap_ros;
        goalIsReached_ = false;

        //debug file
        debugFile_.open("debug.txt");

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
    if(globalPlanIsSet_ && goalIsReached_ == false) {
        // costmap Global Frame ID = odom
        // transform robot pose to geometry_msgs::Stamped
        /*
        tf::Stamped<tf::Pose> tempRobotPose;
        costmap_ros_->getRobotPose(tempRobotPose); //frame id = odom
        tf::poseStampedTFToMsg(tempRobotPose, robotPose_);
        */

        // transform global plan to odom frame
        // TODO: check if transformation is working!
        // ROS_INFO("gbl map frame id: %s", plan_.at(0).header.frame_id.c_str());
        //tf::StampedTransform transform;
        //tf_->lookupTransform("laser","map", ros::Time(0),transform);
        for(std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin(); it != plan_.end(); it++) {
            tf_->waitForTransform("laser", "map", it->header.stamp, ros::Duration(3.0));
            tf_->transformPose("laser", *it, *it);
        }
        // plan_ now in frame laser

        /// calc steerAngle from trajectorie
        // TODO: which point from plan? depending on distance?
        int point = 50; // which point first? distance?
        ROS_INFO("Distance: %f", calcDistance(plan_[0], plan_[point]));
        // +/- M_PI/2? check!
        while(abs(atan2(0-plan_[point].pose.position.x, 0-plan_[point].pose.position.y) + M_PI/2) < 0.3){
            point ++;
        }
        // debug
        ROS_INFO("TLP: Point [%i]  x: %f y: %f z: %f w: %f",
                 point,
                 (float) plan_[point].pose.position.x,
                 (float) plan_[point].pose.position.y,
                 (float) plan_[point].pose.orientation.z,
                 (float) plan_[point].pose.orientation.w);

        //calc steering angle
        float radius;
        float steerAngle = calcAngle(plan_[point].pose.position.x,plan_[point].pose.position.y, radius);
        if(plan_[point].pose.position.y < 0) {
            steerAngle = steerAngle * (-1);
            radius = radius * (-1);
        }
        // obstacle avoidance
        bool objectInPath = false;
        for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it != laserDataTf_.end(); it++){
            objectInPath = checkForObject(radius, it->position.x, it->position.y);
        }
        // search for avoidance path

        if(objectInPath) {
            ROS_INFO("TLP: Object in Path!");
            
            int helper = 1;
            while(true) {
                bool objectLeft = true;
                bool objectRight = true;

                float leftRadius =  radius + helper*0.1;
                float rightRadius = radius - helper*0.1;
                for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it != laserDataTf_.end(); it++){
                    objectLeft = checkForObject(leftRadius, it->position.x, it->position.y);
                    objectRight = checkForObject(rightRadius, it->position.x, it->position.y);
                }
                if(!objectLeft) {
                    cmd_vel.angular.z = atan(WHEELBASE/leftRadius);
                    ROS_INFO("TLP: Alternative right turn!");
                    return false;
                }
                if(!objectRight) {
                    cmd_vel.angular.z = atan(WHEELBASE/rightRadius);
                    ROS_INFO("TLP: Alternative left turn!");
                    return false;
                }
                if(helper == 100) {
                    ROS_INFO("TLP: No alternative path found!");
                    return false;
                }
                helper++;
            }
            //cmd_vel.angular.z = steerAngle*0.6; //remove!
        } else {
            cmd_vel.angular.z = steerAngle*0.6;
        }
    }
    // emergency stop
    bool stopCar = false;
    for(int i = 0; i < 640; i++) {
        //if one front laser scan distance < 0.5 turn off the engine
        if(tlpLaserScan->ranges[i] < 0.0) {
            stopCar = true;
        }
    }
    if(stopCar) {
        cmd_vel.linear.x = 0;
        ROS_INFO("TLP: WARNING! Obstacle in front!");
        return true;
    } else {
        //cmd_vel.linear.x = 1.1 - (cmd_vel.angular.z*M_PI)/4;
        cmd_vel.linear.x =0.2;
	return true;
    }
    // ---
}
bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    ROS_INFO("TLP: new global plan received! length: %i", (int) plan.size());
    globalPlanIsSet_ = true;
    if(plan.size() < 55) {
        goalIsReached_ = true;
    } else {
        goalIsReached_ = false;
    }
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
    for(int i = 200; i < numberLaserPoints-200; i++) {
        //max distance
        if(tlpLaserScan->ranges[i] < 1.5) {
            geometry_msgs::Pose newLaserPoint;
            newLaserPoint.position.x = cos(tlpLaserScan->angle_min + tlpLaserScan->angle_increment*i)*tlpLaserScan->ranges[i];
            newLaserPoint.position.y = sin(tlpLaserScan->angle_min + tlpLaserScan->angle_increment*i)*tlpLaserScan->ranges[i];
            laserDataTf_.push_back(newLaserPoint);
        }
    }

    /*
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

    */
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "laser";
    poseArray.poses = laserDataTf_;
    //ROS_INFO("TLP: publish laserDataTf_ with size: %i", (int) poseArray.poses.size());
    //ROS_INFO("TLP: %i laser objects detected!", (int) laserObjects_.size());
    pubTest_.publish(poseArray);
}

void LocalPlanner::tfRobotPose()
{
}

};
