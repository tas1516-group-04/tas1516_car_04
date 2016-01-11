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

        //parameters
        nodeHandle_.param<double>("/move_base_node/car_width", carwidth_, 0.7);
        nodeHandle_.param<double>("/move_base_node/wheelbase", wheelbase_, 0.3);
        nodeHandle_.param<bool>("/move_base_node/obstacle_avoidance", doObstacleAvoidance_, false);
        nodeHandle_.param<int>("/move_base_node/min_target_point", minTargetPoint_, 50);
        nodeHandle_.param<double>("/move_base_node/steering_angle_parameter", steeringAngleParameter_, 0.6);
        nodeHandle_.param<double>("/move_base_node/laser_max_dist", laserMaxDist_, 2);
        if(doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance active!");
        if(!doObstacleAvoidance_) ROS_INFO("TLP: Obstacle avoidance inactive!");

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
        int point = minTargetPoint_; // which point first? distance?
        ROS_INFO("Distance: %f", calcDistance(plan_[0], plan_[point]));
        // +/- M_PI/2? check!
        while(abs(atan2(0-plan_[point].pose.position.x, 0-plan_[point].pose.position.y) + M_PI/2) < 0.3){
            point ++;
        }
        // debug
        /*ROS_INFO("TLP: TPoint [%i]  x: %f y: %f z: %f w: %f",
                 point,
                 (float) plan_[point].pose.position.x,
                 (float) plan_[point].pose.position.y,
                 (float) plan_[point].pose.orientation.z,
                 (float) plan_[point].pose.orientation.w);
        */
        //calc steering angle
        float steerAngle = calcAngle(plan_[point].pose.position.x,plan_[point].pose.position.y);

        /// obstacle avoidance

        // check if one laser point
        analyzeLaserData(steerAngle);
        bool objectInPath = false;
        for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it != laserDataTf_.end(); it++){
            // returns true if one laser point is in path
            objectInPath = checkForObject(steerAngle, it->position.x, it->position.y);
            if(objectInPath) break;
        }
        // search for avoidance path

        if(objectInPath && doObstacleAvoidance_) {
            ROS_INFO("TLP: Object in path!");

            int helper = 1;
            //steerAngle = 0; //for testing
            while(true) {
                bool objectLeft = true;
                bool objectRight = true;

                float angleInc =  steerAngle + helper*0.05;
                float angleDec =  steerAngle - helper*0.05;
                for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it != laserDataTf_.end(); it++){
                    objectLeft  = checkForObject(angleInc, it->position.x, it->position.y);
                    objectRight = checkForObject(angleDec, it->position.x, it->position.y);
                }
                // decide what to do
                if(!objectLeft) {
                    cmd_vel.angular.z = angleInc*0.3;
                    ROS_INFO("TLP: Alternative: Left turn! Z: %f", (float) cmd_vel.angular.z);
                    break;
                }
                if(!objectRight) {
                    cmd_vel.angular.z = angleDec*0.3;
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
            cmd_vel.angular.z = steerAngle*steeringAngleParameter_;
            cmd_vel.linear.x = 0.2;
        }
    }
    return true;
    // emergency stop
    /*
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
        cmd_vel.linear.x = 0.2;
	      return true;
    }
    */
}
bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    //ROS_INFO("TLP: new global plan received! length: %i", (int) plan.size());
    globalPlanIsSet_ = true;
    if(plan.size() < minTargetPoint_+5) {
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
    for(int i = 100; i < numberLaserPoints-100; i++) {
        //max distance
        if(tlpLaserScan->ranges[i] < laserMaxDist_) {
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
            return true;
        } else {
            // return false otherwise
            return false;
        }
    } else {
        if(pow(x,2) + pow(y-r,2) - pow(r+carwidth_/2,2) >= 0 && pow(x,2) + pow(y-r,2) - pow(r-carwidth_/2,2) <= 0) {
            // return true if object in path
            return true;
        } else {
            // return false otherwise
            return false;
        }
    }
}

};
