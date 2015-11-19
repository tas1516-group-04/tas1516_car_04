#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_pub");


  ros::NodeHandle n;

  
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

  ros::Rate loop_rate(100);

  
  int count = 0;
  while (ros::ok())
  {
    
    geometry_msgs::PoseStamped msg;

    msg.header.frame_id = "map";
    msg.pose.position.x = 20;
    msg.pose.position.y = 19;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 0.01334;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0.9999;

    if (count == 0)
    {
    pose_pub.publish(msg);
    count++;
    }
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
