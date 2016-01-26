#include "ScanFeatures.h"

ScanFeatures::ScanFeatures()
{

}

float ScanFeatures::calcMean(sensor_msgs::LaserScan::ConstPtr &scan_in, int points)
{
    float dist = 0;

    for (int i=0; i < points; i++) {
        dist += scan_in.get()->ranges.at(i);
    }
    dist /= (float)points;
    return dist;
}
