#include "parking_node.h"
//#include "laserhandle.h"
#include "laserscantopointcloud.h"
#include "features.h"
#include "ScanFeatures.h"
#include "../../parking_control/src/control/control.h"      // for speed defines ...

#define MAX_DIST            0.35
#define MIN_DIST            0.3
#define NUM_MEAN_SAMPLES    10

#define MIN_GAP_DEPTH       0.2
#define MAX_GAP_DEPTH       0.35

#define MIN_GAP_LENGTH      0.4
#define MAX_GAP_LENGTH      1.0

#define RANGE_THRESHOLD     0.05


typedef struct {
    float new_dist;
    float old_dist;
    float diff_dist;
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


int main(int argc, char** argv)
{

    ros::init(argc, argv, "parking_node");

    ros::NodeHandle nF;
    ros::NodeHandle nB;
    ros::NodeHandle nVel;

    ros::Publisher cmd_vel_pub = nVel.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // tf listener for robot position:
    tf::TransformListener tf_listener;


    LaserScanToPointCloud lsF(nF, "scan");
    LaserScanToPointCloud lsB(nB, "scan_back");

    sensor_msgs::PointCloud cloudF;
    sensor_msgs::PointCloud cloudB;

    Features features;

    enum states {INIT,
                 FIRST_CORNER_START,
                 FIRST_CORNER_END,
                 SECOND_CORNER_START,
                 SECOND_CORNER_END,
                 START_PARKING };

    int state = INIT;

    dist_t Front;
    dist_t Back;
    robot_t R;
    corners_t C;

    ScanFeatures scan_features;


    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = PARKING_SPEED;
        vel_msg.angular.z = STEERING_RATIO;



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

        // Calc distances for front laser scan, very LEFT ray
        Front.new_dist = scan_features.calcMean(lsF.getScan(), NUM_MEAN_SAMPLES);
        Front.diff_dist = Front.old_dist - Front.new_dist;
        Front.old_dist = Front.new_dist;

        // Calc distances for Back laser scan, very RIGHT ray
        Back.new_dist = scan_features.calcMean(lsB.getScan(), NUM_MEAN_SAMPLES);
        Back.diff_dist = Back.old_dist - Back.new_dist;
        Back.old_dist = Back.new_dist;

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
            if (Front.diff_dist >= RANGE_THRESHOLD) {
                ROS_INFO("dist_diff is in range for first corner end");

                // capture robot position now for second corner start
                C.second.start.xpos = R.xpos;
                C.second.start.ypos = Front.new_dist;

                state = SECOND_CORNER_END;
            }
            break;
        case SECOND_CORNER_END:
            if (Front.diff_dist >= RANGE_THRESHOLD) {
                ROS_INFO("dist_diff is in range for first corner end");

                // capture robot position now for second corner end
                C.second.end.xpos = R.xpos;
                C.second.end.ypos = Front.new_dist;

                state = START_PARKING;
            }
            break;
        case START_PARKING:
            ROS_INFO("All corners detected. Start Parking:");
            ROS_INFO("Set max steering angle and drive backwards...");
            vel_msg.linear.x = - PARKING_SPEED;
            vel_msg.angular.z = MAX_STEERING;

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
