#include "objectavoidance.h"

ObjectAvoidance::ObjectAvoidance(double wheelbase, double carwidth, tf::TransformListener* tf) :
    wheelbase_(wheelbase),
    carwidth_(carwidth),
    tf_(tf)
{
}
geometry_msgs::PoseStamped ObjectAvoidance::doObstacleAvoidance(int targetPoint, std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::Twist& cmd_vel)
{
    std::vector<geometry_msgs::PoseStamped> plan_ = plan;
    for(std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin(); it != plan_.begin()+targetPoint; it++) {
        distToPoint_ = sqrt(pow(targetPoint.pose.position.x,2)+pow(targetPoint.pose.position.y,2));
        if(objectInPath(*it)) {
            ROS_INFO("TLP: Object in Path!");
            // break if object in path
            cmd_vel.linear.x = 0.1;
            return getNewTargetPoint(*it);
        }
        // safety mechanism
        if(it == plan_.end()) break;
    }
    // dont break if no object in path
    cmd_vel.linear.x = 0.5;
    return plan_[targetPoint];
}

bool ObjectAvoidance::objectInPath(geometry_msgs::PoseStamped& targetPoint)
{
    int consecutivePointsInPath = 0;
    double maxConsecutivePointsInPath =  minObjectSize_/(tan(2.8/640)*distToPoint_);
    for(std::vector<geometry_msgs::Point32>::iterator it = laserPoints.points.begin(); it != laserPoints.points.end(); it++){
        // point has to be in path and in range
        if(pointInPath(it->x, it->y, targetPoint)) {
            consecutivePointsInPath++;
        } else {
            if(consecutivePointsInPath > maxConsecutivePointsInPath) return true;
            consecutivePointsInPath = 0;
        }
    }
    return false;
}

bool ObjectAvoidance::pointInPath(double x, double y, geometry_msgs::PoseStamped& targetPoint)
{
    if(pow(x-targetPoint.pose.position.x,2) + pow(y-targetPoint.pose.position.y,2) <= pow(carwidth_/2+0.05,2)){
        // return true if point in path
        return true;
    } else {
        // return false otherwise
        return false;
    }
}

geometry_msgs::PoseStamped ObjectAvoidance::getNewTargetPoint(geometry_msgs::PoseStamped& targetPoint)
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
            return yInc; // which steering parameter?
        }
        if(!objectInPath(yDec)) {
            ROS_INFO("TLP: Alternative: Right turn!");
            return yDec;
        }
    }
    return targetPoint;
}

void ObjectAvoidance::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laser_geometry::LaserProjection projector_;
    if(!tf_->waitForTransform(
                scan->header.frame_id,
                "/laser",
                scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                ros::Duration(1.0))){
        return;
    }
    projector_.transformLaserScanToPointCloud("/laser",*scan, laserPoints,*tf_);
}
