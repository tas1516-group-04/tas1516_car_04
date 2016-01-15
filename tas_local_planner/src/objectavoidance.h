#ifndef OBJECTAVOIDANCE_H
#define OBJECTAVOIDANCE_H

#include <laser_geometry/laser_geometry.h>


class ObjectAvoidance
{
public:
    ObjectAvoidance(double wheelbase, double carwidth);
    double doObstacleAvoidance(double steeringAngle, sensor_msgs::PointCloud &laserPoints);

private:
    bool objectInPath(double steeringAngle, sensor_msgs::PointCloud &laserPoints);
    bool pointInPath(double x, double y, double angle);
    double getNewSteeringAngle(double steeringAngle, sensor_msgs::PointCloud &laserPoints);

    double wheelbase_;
    double carwidth_;
};

#endif // OBJECTAVOIDANCE_H
