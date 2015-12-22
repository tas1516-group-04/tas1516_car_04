#include "parking_node.h"
//#include "laserhandle.h"
//#include "laserscantopointcloud.h"
#include "features.h"
#include "ScanFeatures.h"
#include "../../parking_control/src/control/control.h"      // for speed defines ...
#include <tf/transform_listener.h>
#include <iostream>

using namespace std;

/*
#define MAX_DIST                0.40
#define MIN_DIST                0.30
#define NUM_MEAN_SAMPLES        10

#define MAX_GAP_DEPTH           MAX_DIST
#define MIN_GAP_DEPTH           MIN_DIST

#define MIN_GAP_LENGTH          0.4
#define MAX_GAP_LENGTH          1.0

#define RANGE_THRESHOLD         0.05

#define BACKWARD_THRESHOLD_1    0.2
#define BACKWARD_THRESHOLD_2    0.1

#define LINEAR_SPEED            0.3
*/

typedef struct {
    float new_dist;
    float old_dist;
    float diff_dist;
    float left_dist;
    float right_dist;
    float middle_dist;
} dist_t;

typedef struct {
    float xpos;
    float ypos;
} robot_t;

typedef struct {
    struct {
        struct {
            float xpos;
            float ypos;
        } start;
        struct {
            float xpos;
            float ypos;
        } end;
    } first;
    struct {
        struct {
            float xpos;
            float ypos;
        } start;
        struct {
            float xpos;
            float ypos;
        } end;
    } second;
} corners_t;

// Global vars for laser ranges
dist_t Front;
dist_t Back;

void processLaserScanF(const sensor_msgs::LaserScan::ConstPtr& scan){
    //scan->ranges[] are laser readings
    Front.right_dist = scan->ranges[0];
    Front.middle_dist = scan->ranges[scan->ranges.size() / 2];
    Front.left_dist = scan->ranges[scan->ranges.size() - 1];
    // DEBUG
    //cout << "* Front:" << endl << "  left: " << Front.left_dist << " middle: " << Front.middle_dist << " right: " << Front.right_dist << endl;
}

void processLaserScanB(const sensor_msgs::LaserScan::ConstPtr& scan){
    //scan->ranges[] are laser readings
    Back.left_dist = scan->ranges[0];
    Back.middle_dist = scan->ranges[scan->ranges.size() / 2];
    Back.right_dist = scan->ranges[scan->ranges.size() - 1];
    // DEBUG
    //cout << "* Back:" << endl << "  left: " << Back.left_dist << " middle: " << Back.middle_dist << " right: " << Back.right_dist << endl;
}

int main(int argc, char** argv)
{
    ROS_INFO("Init ros node...");
    ros::init(argc, argv, "parking_node");

    // set node handle namespace
    ros::NodeHandle n("parking");

    ROS_INFO("Add subscribers...");
    ros::Subscriber lsF_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",10, processLaserScanF);
    ros::Subscriber lsB_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_back",10, processLaserScanB);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // tf listener for robot position:
    tf::TransformListener tf_listener;


    // set standard parameters (rosparam only supports double)
    double MAX_DIST =                0.40;
    double MIN_DIST =                0.30;
    double NUM_MEAN_SAMPLES =        10;

    double MAX_GAP_DEPTH =           MAX_DIST;
    double MIN_GAP_DEPTH =           MIN_DIST;

    double MIN_GAP_LENGTH =          0.4;
    double MAX_GAP_LENGTH =          1.0;

    double RANGE_THRESHOLD =         0.05;

    double BACKWARD_THRESHOLD_1 =    0.2;
    double BACKWARD_THRESHOLD_2 =    0.1;

    double LINEAR_SPEED =            0.3;


    // read params from param server
    n.getParam("MAX_DIST", MAX_DIST);
    n.getParam("MIN_DIST", MIN_DIST);

    n.getParam("NUM_MEAN_SAMPLES", NUM_MEAN_SAMPLES);

    n.getParam("MAX_GAP_DEPTH", MAX_GAP_DEPTH);
    n.getParam("MIN_GAP_DEPTH", MIN_GAP_DEPTH);

    n.getParam("MIN_GAP_LENGTH", MIN_GAP_LENGTH);
    n.getParam("MAX_GAP_LENGTH", MAX_GAP_LENGTH);

    n.getParam("RANGE_THRESHOLD", RANGE_THRESHOLD);

    n.getParam("BACKWARD_THRESHOLD_1", BACKWARD_THRESHOLD_1);
    n.getParam("BACKWARD_THRESHOLD_2", BACKWARD_THRESHOLD_2);

    n.getParam("LINEAR_SPEED", LINEAR_SPEED);



    // Default value version
    //n.param<std::string>("default_param", default_param, "default_value");

    enum states {INIT,
                 FIRST_CORNER_START,
                 FIRST_CORNER_END,
                 SECOND_CORNER_START,
                 SECOND_CORNER_END,
                 START_PARKING,
                 PARKING_STATE_1,
                 PARKING_STATE_2,
                 END_PARKING };

    bool display_once = true;

    // set start state
    int state = INIT;

    robot_t R;
    corners_t C;

    ScanFeatures scan_features;

    geometry_msgs::Twist vel_msg;

    ros::Rate loop_rate(100);

    int display_counter = 0;

    // set speed to zero
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    cmd_vel_pub.publish(vel_msg);

    while(ros::ok())
    {
        display_counter = 1;
        if (display_counter%100 == 0) {
            cout << "* Front:" << endl << "  left: " << Front.left_dist << " middle: " << Front.middle_dist << " right: " << Front.right_dist << endl;
            cout << "* Back:" << endl << "  left: " << Back.left_dist << " middle: " << Back.middle_dist << " right: " << Back.right_dist << endl;
        }

        switch(state) {

        case INIT:
            if (display_once) {
                ROS_INFO("Start Parking... ");
                display_once = false;
            }

            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = 0;
            cmd_vel_pub.publish(vel_msg);

            // check if in a certain range next to parking wall
            if (Front.left_dist >= MIN_DIST && Front.left_dist <= MAX_DIST) {
                ROS_INFO("y dist is in range for parking");

                // set robot position in parking coordinate system
                R.xpos = 0;
                R.ypos = Front.new_dist;

                state = FIRST_CORNER_START;
                ROS_INFO("state: FIRST_CORNER_START");
                ROS_INFO("Steer robot along x axis ... ");
            }
            break;

        case FIRST_CORNER_START:
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = Front.left_dist * STEERING_RATIO;   // TODO define steering ratio
            cmd_vel_pub.publish(vel_msg);

            // check if in range to detect first corner start
            if (Front.left_dist >= MIN_GAP_DEPTH && Front.left_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("dist_diff is in range for first corner detection");

                // capture corner position now for first corner start
                C.first.start.xpos = R.xpos;
                C.first.start.ypos = Front.new_dist;

                state = FIRST_CORNER_END;
                ROS_INFO("state: FIRST_CORNER_END");
            }
            break;
        case FIRST_CORNER_END:
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = Front.diff_dist * STEERING_RATIO;   // TODO define steering ratio
            cmd_vel_pub.publish(vel_msg);

            // check if in range to detect first corner end
            if (Front.left_dist <= MIN_GAP_DEPTH) {
                ROS_INFO("Front.left_dist is in range for first corner end");

                // capture robot position now for first corner end
                C.first.end.xpos = R.xpos;
                C.first.end.ypos = Front.new_dist;

                state = SECOND_CORNER_START;
                ROS_INFO("state: SECOND_CORNER_START");
            }
            break;
        case SECOND_CORNER_START:
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = Front.diff_dist * STEERING_RATIO;   // TODO define steering ratio
            cmd_vel_pub.publish(vel_msg);

            if (Back.left_dist >= MIN_GAP_DEPTH && Back.left_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("Back.left_dist is in range for second corner start");

                // capture robot position now for second corner start
                C.second.start.xpos = R.xpos;
                C.second.start.ypos = Front.new_dist;
                state = START_PARKING;
                ROS_INFO("state: START_PARKING");
                ROS_INFO("Driving backwards with min steering angle...");
            }
            break;
        /*
        case SECOND_CORNER_END:
            if (Front.diff_dist >= RANGE_THRESHOLD) {
                ROS_INFO("dist_diff is in range for second corner end");

                // capture robot position now for second corner end
                C.second.end.xpos = R.xpos;
                C.second.end.ypos = Front.new_dist;

                state = START_PARKING;
            }
            break;

        */
        case START_PARKING:

            vel_msg.linear.x = - LINEAR_SPEED;
            vel_msg.angular.z = - MAX_STEERING;
            cmd_vel_pub.publish(vel_msg);

            // check if first backward distance reached
            if (Back.middle_dist <= BACKWARD_THRESHOLD_1) {
                state = PARKING_STATE_1;
                ROS_INFO("state: PARKING_STATE_1");
                ROS_INFO("Steering reversal point reached!");
                ROS_INFO("Driving Backwards with max steering angle...");
            }

            break;

        case PARKING_STATE_1:

            vel_msg.linear.x = - LINEAR_SPEED;
            vel_msg.angular.z = MAX_STEERING;
            cmd_vel_pub.publish(vel_msg);

            // check if first backward distance reached
            if (Back.middle_dist <= BACKWARD_THRESHOLD_2) {
                state = PARKING_STATE_2;
                ROS_INFO("state:PARKING_STATE_2");
            }

            break;

        case PARKING_STATE_2:
            if (display_once) {
                ROS_INFO("Minimum distance to back reached!");
                ROS_INFO("Driving forward for 0.1 m if way is cleared");

                display_once = false;
            }

            // TODO add ray component for middle ray here
            if (Front.middle_dist >= 0.2) {
                vel_msg.linear.x = LINEAR_SPEED;
                vel_msg.angular.z = 0;
                cmd_vel_pub.publish(vel_msg);
            }

            // check if forward min distance reached
            if (Front.middle_dist <= 0.2) {
                state = END_PARKING;
                ROS_INFO("state: END_PARKING");
            }

            break;

        case END_PARKING:
            if (display_once) {
                ROS_INFO("Parking Finished. Setting speed to zero!");
                display_once = false;
            }

            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            cmd_vel_pub.publish(vel_msg);

            ROS_INFO("Finished! Exiting...");
            exit(EXIT_SUCCESS);
            break;

         default:
            // set speed to zero
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            ROS_INFO ("Aborted state machine. Set speed to zero");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }




    return 0;
}
