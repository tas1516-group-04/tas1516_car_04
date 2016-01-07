#ifndef DEBUGCLASS_H
#define DEBUGCLASS_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <cmath>
#include <vector>
#include <fstream>

class DebugClass
{
private:


public:
    DebugClass(ros::NodeHandle& nh, const tf::TransformListener *tf) : nh(nh), tf(tf)
    {
        std::cout << "\n\nDEBUG CLASS WORKING" << std::endl;
        std::cout << &nh << std::endl;

    }

    void create_publisher()
    {
       std::cout << "\n HELLO PUBLISHER" << std::endl;
       cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_velocity", 1);
       std::cout << "\n HELLO PUBLISHER" << std::endl;
    }

    ros::NodeHandle& nh;
    const tf::TransformListener *tf;

    ros::Subscriber cmd_subscriber;
    ros::Publisher cmd_publisher;
};

#endif // DEBUGCLASS_H
