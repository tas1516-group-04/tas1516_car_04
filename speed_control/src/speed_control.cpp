// Author: Fabian Lechner

// Main program for speed control. Constructs speed controller and handles IO with the ROS system.
// Subscibed to:    move_base_node/NavfnROS/plan
// Pubishes to:     vel
// The speed controller is implemented as a class in speed_controller.cpp


#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <fstream>
#include <tf/transform_listener.h>

#include "speed_controller.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "curve_detect");
    ros::NodeHandle n;

    const tf::TransformListener tf_listener;
    // Construct and initialize speed controller
    SpeedController sc(&tf_listener);
    sc.set_parameters(n);
    // Subscribe to global plan topic
    ros::Subscriber plan_subscriber = n.subscribe("move_base_node/NavfnROS/plan", 10,
                                                &SpeedController::planCallback, &sc);
    // Advertise vel topic
    ros::Publisher vel_publisher = n.advertise<geometry_msgs::Twist>("vel", 100);

    ros::Rate loop_rate(20);
    ROS_INFO("SpeedController running. Entering loop.\n");
    while (ros::ok())
    {
        // Let speed controller compute velocity and publish the velocity with the vel topic
        geometry_msgs::Twist speed;
        speed.linear.x = sc.calcSpeed();
        vel_publisher.publish(speed);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
