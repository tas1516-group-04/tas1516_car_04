#include "ros/ros.h"
#include "nav_msgs/Path.h"

void planReceivedCallback(const nav_msgs::Path::ConstPtr& path)
{
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "curve_detect");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("move_base_node/NavfnROS/plan", 10, planReceivedCallback);

  ros::spin();

  return 0;
}