#ifndef FEATURES_H
#define FEATURES_H

#include <sensor_msgs/PointCloud.h>

class Features
{
private:
    geometry_msgs::Point32 old_point;
    sensor_msgs::PointCloud mean_cloud;
public:
    float step;
    Features();
    void setStepThreshold(float step);
    double calcStep(sensor_msgs::PointCloud &cloud);
    bool getStepThreshold();
};

#endif // FEATURES_H
