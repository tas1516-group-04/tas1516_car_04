#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <fstream>


using namespace std;

void planReceivedCallback(const nav_msgs::Path::ConstPtr& path)
{
  fstream logfile("/home/tas_group_04/catkin_ws/src/tas1516_car_04/curve_detect/log/curve_detect.log", ios::in | ios::out | ios::app);
  if (logfile.is_open())
  {

    logfile << "Timestamp: " << path->header.stamp << endl;
    for (geometry_msgs::PoseStamped point : path->poses)
    {
        logfile << point.pose.position.x << ","
                << point.pose.position.y << ","
                << point.pose.position.z << ","
                << point.pose.orientation.w << ","
                << point.pose.orientation.x << ","
                << point.pose.orientation.y << ","
                << point.pose.orientation.z << endl;
    }
    logfile.close();
  }
  else cout << "Unable to open file";

  ROS_INFO("%lu", path->poses.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "curve_detect");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("move_base_node/NavfnROS/plan", 10, planReceivedCallback);

  ros::spin();

  return 0;
}
