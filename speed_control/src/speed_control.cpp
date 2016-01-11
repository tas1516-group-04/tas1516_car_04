#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <fstream>
#include <tf/transform_listener.h>

#include "speed_controller.h"


using namespace std;

//void planReceivedCallback(const nav_msgs::Path::ConstPtr &path) {
//    fstream logfile("/home/tas_group_04/catkin_ws/src/tas1516_car_04/curve_detect/log/curve_detect.log",
//                    ios::in | ios::out | ios::app);
//    if (logfile.is_open()) {

//        logfile << "\nTimestamp: " << path->header.stamp << endl;
//        for (geometry_msgs::PoseStamped point : path->poses) {
//            logfile << point.pose.position.x << ","
//            << point.pose.position.y << ","
//            << point.pose.position.z << ","
//            << point.pose.orientation.w << ","
//            << point.pose.orientation.x << ","
//            << point.pose.orientation.y << ","
//            << point.pose.orientation.z << endl;
//        }
//        logfile.close();
//    }
//    else cout << "Unable to open file";

//    ROS_INFO("%lu", path->poses.size()    // Need at least some elements in path
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "curve_detect");
    ros::NodeHandle n;

    const tf::TransformListener tf_listener;
    SpeedController sc(&tf_listener);
    sc.set_parameters(n);
    ros::Subscriber plan_subscriber = n.subscribe("move_base_node/NavfnROS/plan", 10,
                                                &SpeedController::planCallback, &sc);
    ros::Publisher vel_publisher = n.advertise<geometry_msgs::Twist>("vel", 100);

    ros::Rate loop_rate(20);
    ROS_INFO("SpeedController running. Entering loop.\n");
    while (ros::ok())
    {
        geometry_msgs::Twist speed;
        speed.linear.x = sc.calcSpeed();
        vel_publisher.publish(speed);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
