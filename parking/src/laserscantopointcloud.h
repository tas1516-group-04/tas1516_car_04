// Author: Thomas Eiband
// Obsolete class for point cloud based feature detection
// Provides node handle for laser scan topics and extracted point cloud

#ifndef LASERSCANTOPOINTCLOUD_H
#define LASERSCANTOPOINTCLOUD_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


#define PI                  3.14159265
#define CAR_LENGTH          0.355   // wrong??

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  sensor_msgs::PointCloud cloud;

  // supply laser scan as member variable
  sensor_msgs::LaserScan::ConstPtr scan_in;

  LaserScanToPointCloud(ros::NodeHandle n, const char* laser_topic_name) :
    n_(n),
    laser_sub_(n_, laser_topic_name, 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/thomas_cloud",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    this->scan_in = scan_in;

    try
    {
        projector_.transformLaserScanToPointCloud("base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    // Do something with cloud.

    scan_pub_.publish(cloud);

  }

  sensor_msgs::PointCloud getCloud()
  {
      return cloud;
  }

  sensor_msgs::LaserScan::ConstPtr &getScan()
  {
      return scan_in;
  }
};

#endif // LASERSCANTOPOINTCLOUD_H
