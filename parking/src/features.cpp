#include "features.h"

Features::Features()
{
    step = 9999;
    old_point.x = 0;
    old_point.y = 0;
}

void Features::setStepThreshold(float step)
{
    this->step = step;
}

double Features::calcStep(sensor_msgs::PointCloud &cloud)
{
    geometry_msgs::Point32 diff;
    diff.x = old_point.x - (cloud.points[0].x   );
    diff.y = old_point.y - (cloud.points[0].y   );
    old_point = cloud.points[0];
    return pow(diff.x, 2) + pow(diff.y, 2);
}

bool Features::getStepThreshold()
{

}

// TODO add closest point from cloud method

// TODO add mean points function (over 10 times mean value)
