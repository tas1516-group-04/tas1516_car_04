#include "objectavoidance.h"

ObjectAvoidance::ObjectAvoidance(double wheelbase, double carwidth, double corridorWidth, double minDistance) :
    wheelbase_(wheelbase),
    carwidth_(carwidth),
    corridorWidth_(corridorWidth),
    minDistance_(minDistance)
{
    subScan_ = nodeHandle_.subscribe("scan", 1000, &ObjectAvoidance::scanCallback, this);
    pubScanTf_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("scanTf", 1000);
    nodeHandle_.param<double>("/move_base_node/min_object_size", minObjectSize_, 1);
}
geometry_msgs::PoseStamped ObjectAvoidance::doObstacleAvoidance(int targetPoint, std::vector<geometry_msgs::PoseStamped> plan, geometry_msgs::Twist& cmd_vel)
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

bool ObjectAvoidance::objectInPath(geometry_msgs::PoseStamped targetPoint)
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

bool ObjectAvoidance::pointInPath(double x, double y, geometry_msgs::PoseStamped targetPoint)
{
    if(pow(x-targetPoint.pose.position.x,2) + pow(y-targetPoint.pose.position.y,2) <= pow(carwidth_/2+0.05,2)){
        // return true if point in path
        return true;
    } else {
        // return false otherwise
        return false;
    }
}

geometry_msgs::PoseStamped ObjectAvoidance::getNewTargetPoint(geometry_msgs::PoseStamped targetPoint)
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
            break;
        }
        if(!objectInPath(yDec)) {
            ROS_INFO("TLP: Alternative: Right turn!");
            ROS_INFO("TLP: o.a. | x: %f | y: %f ",
                     yDec.pose.position.x,
                     yDec.pose.position.y);
            return yDec;
            break;
        }
    }
    return targetPoint;
}

void ObjectAvoidance::filterLaserScan()
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
    sensor_msgs::PointCloud pcl;
    pcl.header.frame_id = "laser";
    pcl.points = laserPoints;
    pubScanTf_.publish(pcl);
    //ROS_INFO("TLP: %i laser points", (int) laserPoints.size());
}

void ObjectAvoidance::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    scan_ = scan;
}
