#include "objectavoidance.h"

ObjectAvoidance::ObjectAvoidance(double wheelbase, double carwidth, tf::TransformListener* tf) :
    wheelbase_(wheelbase),
    carwidth_(carwidth),
    tf_(tf)
{
}

double ObjectAvoidance::doObstacleAvoidance(double steeringAngle)
{
    if(objectInPath(steeringAngle)) {
        ROS_INFO("TLP: Object in path!");
        return getNewSteeringAngle(steeringAngle);
    } else {
        return steeringAngle;
    }
}

bool ObjectAvoidance::objectInPath(double steeringAngle)
{
    transformLaserScan();
    int consecutivePointsInPath = 0;
    /*
    for(std::vector<geometry_msgs::Point32>::iterator it = laserPoints.points.begin(); it != laserPoints.points.end(); it++){
        // point has to be in path and in range
        if(pointInPath(it->x, it->y, steeringAngle) && pow(it->x,2) + pow(it->y,2) < 1.5) {
            consecutivePointsInPath++;
        } else {
            consecutivePointsInPath = 0;
        }
        if(consecutivePointsInPath > minObjectSize_) return true;
    }
    */
    for(std::vector<geometry_msgs::Pose>::iterator it = laserDataTf_.begin(); it != laserDataTf_.end(); it++){
        // point has to be in path and in range
        if(pointInPath(it->position.x, it->position.y, steeringAngle) && it->position.z < 1.5) {
            consecutivePointsInPath++;
        } else {
            consecutivePointsInPath = 0;
        }
        if(consecutivePointsInPath > minObjectSize_) return true;
    }
    return false;
}

bool ObjectAvoidance::pointInPath(double x, double y, double angle)
{
    if(pow(x+wheelbase_,2) + pow(y-yM,2) - pow(radius-carwidth_/2,2) >= 0
            && pow(x+wheelbase_,2) + pow(y-yM,2) - pow(radius+carwidth_/2,2) <= 0) {
        // return true if point in path
        return true;
    } else {
        // return false otherwise
        return false;
    }
}

double ObjectAvoidance::getNewSteeringAngle(double steeringAngle)
{
    for(int i = 1; i < 10; i++) {
        float angleInc = steeringAngle + i * 0.05;
        float angleDec = steeringAngle - i * 0.05;

        // decide what to do
        if(!objectInPath(angleInc)) {
            ROS_INFO("TLP: Alternative: Left turn! Z: %f", (float) angleInc);
            return angleInc; // which steering parameter?
        }
        if(!objectInPath(angleDec)) {
            ROS_INFO("TLP: Alternative: Right turn! Z: %f",(float) angleDec);
            return angleDec;
        }
    }
    return steeringAngle;
}

void ObjectAvoidance::transformLaserScan(){
    laserDataTf_.clear();
    int numberLaserPoints = (int) ( (abs(laserScan->angle_min) + abs(laserScan->angle_max))/laserScan->angle_increment);
    for(int i = 0; i < numberLaserPoints; i++) {
        //max distance
        geometry_msgs::Pose newLaserPoint;
        newLaserPoint.position.x = cos(laserScan->angle_min + laserScan->angle_increment*i)*laserScan->ranges[i];
        newLaserPoint.position.y = sin(laserScan->angle_min + laserScan->angle_increment*i)*laserScan->ranges[i];
        newLaserPoint.position.z = laserScan->ranges[i];
        laserDataTf_.push_back(newLaserPoint);
    }
}

void ObjectAvoidance::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laserScan = scan;
    /*
    laser_geometry::LaserProjection projector_;
    if(!tf_->waitForTransform(
                scan->header.frame_id,
                "/laser",
                scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                ros::Duration(1.0)))        return;
    }
    projector_.transformLaserScanToPointCloud("/laser",*scan, laserPoints,*tf_);
    */
}
