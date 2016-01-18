#ifndef SCAN_FEATURES_H
#define SCAN_FEATURES_H

#include <laser_geometry/laser_geometry.h>

class ScanFeatures
{
public:
    ScanFeatures();

    ~ScanFeatures() {}
    float calcMean(sensor_msgs::LaserScan::ConstPtr &scan_in, int points);
};

#endif // SCAN_FEATURES_H
