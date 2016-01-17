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
    int consecutivePointsInPath = 0;
    for(std::vector<geometry_msgs::Point32>::iterator it = laserPoints.points.begin(); it != laserPoints.points.end(); it++){
        // point has to be in path and in range
        if(pointInPath(it->x, it->y, steeringAngle) && pow(it->x,2) + pow(it->y,2) < 1.5) {
            consecutivePointsInPath++;
        } else {
            consecutivePointsInPath = 0;
        }
        if(consecutivePointsInPath > 8) return true;
    }
    return false;
}

bool ObjectAvoidance::pointInPath(double x, double y, double angle)
{
    double maxRange = 1.5;
    double range = sqrt(pow(x,2) + pow(y,2));
    if(pow(x+wheelbase_,2) + pow(y-yM,2) - pow(radius-carwidth_/2,2) >= 0
            && pow(x+wheelbase_,2) + pow(y-yM,2) - pow(radius+carwidth_/2,2) <= 0
            && range < maxRange) {
        // return true if point in path
        return true;
    } else {
        // return false otherwise
        return false;
    }
}

double ObjectAvoidance::getNewSteeringAngle(double steeringAngle)
{
    for(int i = 1; i < 100; i++) {
        float angleInc = steeringAngle + i * 0.01;
        float angleDec = steeringAngle - i * 0.01;

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
