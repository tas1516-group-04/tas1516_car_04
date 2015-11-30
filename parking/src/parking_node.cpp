#include "parking_node.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);

  ros::spin();

  return 0;
}
