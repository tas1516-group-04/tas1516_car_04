#include "objectavoidance.h"

ObjectAvoidance::ObjectAvoidance(double wheelbase, double carwidth) :
    wheelbase_(wheelbase),
    carwidth_(carwidth)
{
}

double ObjectAvoidance::doObstacleAvoidance(double steeringAngle, sensor_msgs::PointCloud& laserPoints)
{
    if(objectInPath(steeringAngle, laserPoints)) {
        ROS_INFO("TLP: Object in path!");
        return getNewSteeringAngle(steeringAngle, laserPoints);
    } else {
        return steeringAngle;
    }
}

bool ObjectAvoidance::objectInPath(double steeringAngle, sensor_msgs::PointCloud& laserPoints)
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
    double r = wheelbase_/tan(angle);
    double maxRange = 1/(1+r)*(0.3 - 1)+1;
    double range = sqrt(pow(x,2) + pow(y,2));
    if(r > 0) {
        // x^2 +(y-r-w/2)^2-r^2
        if(pow(x,2) + pow(y-r,2) - pow(r-carwidth_/2,2) >= 0
                && pow(x,2) + pow(y-r,2) -pow(r+carwidth_/2,2) <= 0
                && range < maxRange) {
            // return true if point in path
            return true;
        } else {
            // return false otherwise
            return false;
        }
    } else {
        if(pow(x,2) + pow(y-r,2) - pow(r+carwidth_/2,2) >= 0
                && pow(x,2) + pow(y-r,2) - pow(r-carwidth_/2,2) <= 0
                && range < maxRange) {
            // return true if point in path
            return true;
        } else {
            // return false otherwise
            return false;
        }
    }
}

double ObjectAvoidance::getNewSteeringAngle(double steeringAngle, sensor_msgs::PointCloud& laserPoints)
{
    for(int i = 1; i < 100; i++) {
        float angleInc = steeringAngle + i * 0.01;
        float angleDec = steeringAngle - i * 0.01;

        // decide what to do
        if(!objectInPath(angleInc, laserPoints)) {
            ROS_INFO("TLP: Alternative: Left turn! Z: %f", (float) angleInc);
            return angleInc; // which steering parameter?
        }
        if(!objectInPath(angleDec, laserPoints)) {
            ROS_INFO("TLP: Alternative: Right turn! Z: %f",(float) angleDec);
            return angleDec;
        }
    }
    return steeringAngle;
}
