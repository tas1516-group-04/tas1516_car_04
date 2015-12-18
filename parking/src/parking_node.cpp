#include "parking_node.h"
//#include "laserhandle.h"
//#include "laserscantopointcloud.h"
#include "features.h"
#include "ScanFeatures.h"
#include "../../parking_control/src/control/control.h"      // for speed defines ...
#include <tf/transform_listener.h>
#include <iostream>

using namespace std;

#define MAX_DIST                0.35
#define MIN_DIST                0.3
#define NUM_MEAN_SAMPLES        10

#define MIN_GAP_DEPTH           0.2
#define MAX_GAP_DEPTH           0.35

#define MIN_GAP_LENGTH          0.4
#define MAX_GAP_LENGTH          1.0

#define RANGE_THRESHOLD         0.05

#define BACKWARD_THRESHOLD_1    0.5
#define BACKWARD_THRESHOLD_2    0.1


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


dist_t Front;
dist_t Back;

void processLaserScanF(const sensor_msgs::LaserScan::ConstPtr& scan){
    //scan->ranges[] are laser readings
    Front.left_dist = scan->ranges[0];
    Front.middle_dist = scan->ranges[scan->ranges.size() / 2];
    Front.right_dist = scan->ranges[scan->ranges.size() - 1];
    // DEBUG
    cout << "* Front:" << endl << "  left: " << Front.left_dist << " middle: " << Front.middle_dist << " right: " << endl;
}

void processLaserScanB(const sensor_msgs::LaserScan::ConstPtr& scan){
    //scan->ranges[] are laser readings
    Front.left_dist = scan->ranges[0];
    Front.middle_dist = scan->ranges[scan->ranges.size() / 2];
    Front.right_dist = scan->ranges[scan->ranges.size() - 1];
}

int main(int argc, char** argv)
{
    ROS_INFO("Init ros node...");
    ros::init(argc, argv, "parking_node");

    ros::NodeHandle nF;
    ros::NodeHandle nB;

    ros::NodeHandle nVel;

    ROS_INFO("Add subscribers...");
    ros::Subscriber lsF_sub = nF.subscribe<sensor_msgs::LaserScan>("/scan",10, processLaserScanF);
    ros::Subscriber lsB_sub = nB.subscribe<sensor_msgs::LaserScan>("/scan_back",10, processLaserScanB);

    ros::Publisher cmd_vel_pub = nVel.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // tf listener for robot position:
    tf::TransformListener tf_listener;

    Features features;

    enum states {INIT,
                 FIRST_CORNER_START,
                 FIRST_CORNER_END,
                 SECOND_CORNER_START,
                 SECOND_CORNER_END,
                 START_PARKING,
                 PARKING_STATE_1,
                 PARKING_STATE_2,
                 END_PARKING };

    int state = INIT;

    robot_t R;
    corners_t C;

    ScanFeatures scan_features;


    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = PARKING_SPEED;
        vel_msg.angular.z = 0;



        /*
        tf::StampedTransform transform;
        try{
          tf_listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);   // TODO change to no transform of robot coordinates
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        // update robot position;
        R.xpos = transform.getOrigin().x();
        R.ypos = transform.getOrigin().y();
        */

        /*
        // Calc distances for front laser scan, very LEFT ray
        Front.new_dist = yposF;
        Front.diff_dist = Front.old_dist - Front.new_dist;
        Front.old_dist = Front.new_dist;

        // Calc distances for Back laser scan, very RIGHT ray
        Back.new_dist = yposB;
        Back.diff_dist = Back.old_dist - Back.new_dist;
        Back.old_dist = Back.new_dist;
        */

        switch(state) {

        case INIT:
            // check if in a certain range next to parking wall
            if (Front.new_dist >= MIN_DIST && Front.new_dist <= MAX_DIST) {
                ROS_INFO("new_dist is in range for parking");

                // set robot position in parking coordinate system
                R.xpos = 0;
                R.ypos = Front.new_dist;

                state = FIRST_CORNER_START;
            }
            break;

        case FIRST_CORNER_START:
            // steer robot
            vel_msg.linear.x = PARKING_SPEED;
            vel_msg.angular.z = Front.diff_dist * STEERING_RATIO;   // TODO define steering ratio

            // check if in range to detect first corner start
            if (Front.new_dist >= MIN_GAP_DEPTH && Front.new_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("dist_diff is in range for first corner detection");

                // capture corner position now for first corner start
                C.first.start.xpos = R.xpos;
                C.first.start.ypos = Front.new_dist;

                state = FIRST_CORNER_END;
            }
            break;
        case FIRST_CORNER_END:
            // steer robot
            vel_msg.linear.x = PARKING_SPEED;
            vel_msg.angular.z = Front.diff_dist * STEERING_RATIO;   // TODO define steering ratio
            // check if in range to detect first corner end
            if (Front.new_dist <= MIN_GAP_DEPTH) {
                ROS_INFO("dist_diff is in range for first corner end");

                // capture robot position now for first corner end
                C.first.end.xpos = R.xpos;
                C.first.end.ypos = Front.new_dist;

                state = SECOND_CORNER_START;
            }
            break;
        case SECOND_CORNER_START:
            // steer robot
            vel_msg.linear.x = PARKING_SPEED;
            vel_msg.angular.z = Front.diff_dist * STEERING_RATIO;   // TODO define steering ratio

            if (Back.new_dist >= MIN_GAP_DEPTH && Back.new_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("dist_diff is in range for second corner start");

                // capture robot position now for second corner start
                C.second.start.xpos = R.xpos;
                C.second.start.ypos = Front.new_dist;

                state = SECOND_CORNER_END;
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
            ROS_INFO("All corners detected. Start Parking:");
            ROS_INFO("Driving backwards with max steering angle...");

            vel_msg.linear.x = - PARKING_SPEED;
            vel_msg.angular.z = MAX_STEERING;

            // check if first backward distance reached
            if (Back.new_dist <= BACKWARD_THRESHOLD_1) {
                state = PARKING_STATE_1;
            }

            break;

        case PARKING_STATE_1:
            ROS_INFO("Steering reversal point reached!");
            ROS_INFO("Driving Backwards with min. steering angle...");

            vel_msg.linear.x = - PARKING_SPEED;
            vel_msg.angular.z = - MAX_STEERING;

            // check if first backward distance reached
            if (Back.new_dist <= BACKWARD_THRESHOLD_2) {
                state = PARKING_STATE_2;
            }

            break;

        case PARKING_STATE_2:
            ROS_INFO("Minimum distance to back reached!");
            ROS_INFO("Driving forward for 0.1 m if way is cleared");

            // TODO add ray component for middle ray here
            if (Front.new_dist >= 0.2) {
                vel_msg.linear.x = PARKING_SPEED;
                vel_msg.angular.z = 0;
            }

            // check if forward min distance reached
            if (Front.new_dist <= 0.2) {
                state = END_PARKING;
            }

            break;

        case END_PARKING:
            ROS_INFO("Parking Finished. Setting speed to zero!");
            vel_msg.linear.x = PARKING_SPEED;
            vel_msg.angular.z = 0;

            ROS_INFO("Finished! Exiting...");
            exit(EXIT_SUCCESS);
            break;

         default:

            // set speed to zero
            vel_msg.linear.x = PARKING_SPEED;
            vel_msg.angular.z = STEERING_RATIO;
            ROS_INFO ("Aborted state machine. Set speed to zero");
        }

        // pusblish new cmd_vel
        cmd_vel_pub.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }




    return 0;
}
