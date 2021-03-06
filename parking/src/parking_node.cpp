#include "parking_node.h"
//#include "laserhandle.h"
//#include "laserscantopointcloud.h"
#include "features.h"
#include "ScanFeatures.h"
#include "../../parking_control/src/control/control.h"      // for speed defines ...
#include <tf/transform_listener.h>
#include <iostream>
#include <sensor_msgs/Imu.h>

// for file printing
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <time.h>
#include <string>

#define MAX_DATE 12

using namespace std;



std::string get_date(void)
{
   time_t now;
   char the_date[MAX_DATE];

   the_date[0] = '\0';

   now = time(NULL);

   if (now != -1)
   {
      strftime(the_date, MAX_DATE, "%d_%m_%Y", gmtime(&now));
   }

   return std::string(the_date);
}

typedef struct {
    float new_dist;
    float old_dist;
    float diff_dist;
    float left_dist;
    float right_dist;
    float middle_dist;
    float half_left_dist;
} dist_t;

typedef struct {
    float xpos;
    float ypos;
    float yaw;
} robot_t;

// Global vars for laser ranges
dist_t Front;
dist_t Back;

void processLaserScanF(const sensor_msgs::LaserScan::ConstPtr& scan){
    //scan->ranges[] are laser readings
    Front.right_dist = scan->ranges[0];
    Front.middle_dist = scan->ranges[scan->ranges.size() / 2];
    Front.left_dist = scan->ranges[scan->ranges.size() - 1];
    Front.half_left_dist = scan->ranges[scan->ranges.size() - ((scan->ranges.size() * 3) / 8)];
    // DEBUG
    //cout << "* Front:" << endl << "  left: " << Front.left_dist << " middle: " << Front.middle_dist << " right: " << Front.right_dist << endl;
}

void processLaserScanB(const sensor_msgs::LaserScan::ConstPtr& scan){
    //scan->ranges[] are laser readings
    Back.left_dist = scan->ranges[0];
    Back.middle_dist = scan->ranges[scan->ranges.size() / 2];
    Back.right_dist = scan->ranges[scan->ranges.size() - 1];
    Back.half_left_dist = scan->ranges[ (scan->ranges.size() * 3) / 8 ];
    // DEBUG
    //cout << "* Back:" << endl << "  left: " << Back.left_dist << " middle: " << Back.middle_dist << " right: " << Back.right_dist << endl;
}

// global IMU orientations
double roll, pitch, yaw;

void processImu(const sensor_msgs::Imu::ConstPtr& imu)
{
    //geometry_msgs::Twist vel;
    tf::Quaternion bq(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
    // double roll, pitch, yaw;
    tf::Matrix3x3 (bq).getRPY(roll, pitch, yaw);
    // imu_vel.angular.z = roll;
    // imu_vel.linear.x = pitch;
    // pub.publish(vel)
}

float angularControl(double min, double max, float angular_speed)
{
    //return angular_speed * (Front.left_dist - max);
    
    if (Front.left_dist > max)
        return angular_speed;
    else if ((Front.left_dist < min))
        return (- angular_speed);
    else
        return 0.00001; // return for neutral steering
    
}

// P controller for angular control
float angularControlP (double min, double max, float angular_speed)
{
    const float P = 20;     // controller gain
    return angular_speed * P * (Front.left_dist - ((min+max) / 2));
}


void printLaserRanges()
{
    cout << "* Front:" << endl << "  left: " << Front.left_dist << " middle: " << Front.middle_dist << " right: " << Front.right_dist << endl;
    cout << "* Back:" << endl << "  left: " << Back.left_dist << " middle: " << Back.middle_dist << " right: " << Back.right_dist << endl;
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
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data",10, processImu);


    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // tf listener for robot position:
    // tf::TransformListener tf_listener;

    // declare parameter variables
    double MAX_DIST,
            MIN_DIST,
            NUM_MEAN_SAMPLES,
            MAX_GAP_DEPTH,
            MIN_GAP_DEPTH,
            MAX_GAP_LENGTH,
            MIN_GAP_LENGTH,
            RANGE_THRESHOLD,
            FORWARD_THRESHOLD_1,
            FORWARD_THRESHOLD_2,
            BACKWARD_THRESHOLD_1,
            BACKWARD_THRESHOLD_2,
            LINEAR_SPEED,
            ANGULAR_SPEED,
            YAW_THRESHOLD,
            STEERING_FIRST,
            BACKWARD_SPEED_1,
            BACKWARD_SPEED_2;

    ROS_INFO("Parsing parameters...");

    n.param<double>("MAX_DIST", MAX_DIST, 0.0);
    n.param<double>("MIN_DIST", MIN_DIST, 0.0);
    n.param<double>("NUM_MEAN_SAMPLES", NUM_MEAN_SAMPLES, 0.0);
    n.param<double>("MAX_GAP_DEPTH", MAX_GAP_DEPTH, 0.0);
    n.param<double>("MIN_GAP_DEPTH", MIN_GAP_DEPTH, 0.0);
    n.param<double>("MAX_GAP_LENGTH", MAX_GAP_LENGTH, 0.0);
    n.param<double>("MIN_GAP_LENGTH", MIN_GAP_LENGTH, 0.0);
    n.param<double>("RANGE_THRESHOLD", RANGE_THRESHOLD, 0.0);
    n.param<double>("FORWARD_THRESHOLD_1", FORWARD_THRESHOLD_1, 0.0);
    n.param<double>("FORWARD_THRESHOLD_2", FORWARD_THRESHOLD_2, 0.0);
    n.param<double>("BACKWARD_THRESHOLD_1", BACKWARD_THRESHOLD_1, 0.0);
    n.param<double>("BACKWARD_THRESHOLD_2", BACKWARD_THRESHOLD_2, 0.0);
    n.param<double>("LINEAR_SPEED", LINEAR_SPEED, 0.0);
    n.param<double>("ANGULAR_SPEED", ANGULAR_SPEED, 0.0);
    n.param<double>("YAW_THRESHOLD", YAW_THRESHOLD, 0.0);
    n.param<double>("STEERING_FIRST", STEERING_FIRST, 0.0);
    n.param<double>("BACKWARD_SPEED_1", BACKWARD_SPEED_1, 0.0);
    n.param<double>("BACKWARD_SPEED_2", BACKWARD_SPEED_2, 0.0);

    if (!(MAX_DIST &&
          MIN_DIST &&
          NUM_MEAN_SAMPLES &&
          MAX_GAP_DEPTH &&
          MIN_GAP_DEPTH &&
          MAX_GAP_LENGTH &&
          MIN_GAP_LENGTH &&
          RANGE_THRESHOLD &&
          FORWARD_THRESHOLD_1 &&
          FORWARD_THRESHOLD_2 &&
          BACKWARD_THRESHOLD_1 &&
          BACKWARD_THRESHOLD_2 &&
          LINEAR_SPEED &&
          ANGULAR_SPEED &&
          YAW_THRESHOLD &&
          STEERING_FIRST &&
          BACKWARD_SPEED_1 &&
          BACKWARD_SPEED_2) ) {
        ROS_ERROR("Could not find all parameters from the server. Is \"run_parking.launch\" already launched?");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("done");

    // declare parking states
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

    geometry_msgs::Twist vel_msg;

    ros::Rate loop_rate(1000);

    int display_counter = 0;

    // set speed to zero
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    cmd_vel_pub.publish(vel_msg);


    std::vector<float> yaw_vec;

    while(ros::ok())
    {
        display_counter++;
        if (display_counter%1000 == 0) {
            //cout << "yaw: " << yaw << endl;
            // printLaserRanges();
        }

        if (display_counter%100 == 0)
            yaw_vec.push_back(yaw);

        switch(state) {

        case INIT:
            if (display_once) {
                ROS_INFO("Start Parking... ");
                display_once = false;
            }
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = angularControlP(MIN_DIST, MAX_DIST, ANGULAR_SPEED);
            cmd_vel_pub.publish(vel_msg);

            // check if in a certain range next to parking wall
            if (Front.left_dist >= MIN_GAP_DEPTH && Front.left_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("Front.left_dist is in range for parking");
                cout << "Front.left_dist " << Front.left_dist << endl;

                // set robot position in parking coordinate system
                R.xpos = 0;
                R.ypos = Front.new_dist;

                cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

                state = FIRST_CORNER_START;
                ROS_INFO("state: FIRST_CORNER_START");
                ROS_INFO("Steer robot along x axis ... ");
            }
            break;

        case FIRST_CORNER_START:
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;

            // check if distance to first corner large enough for active angular controller
            //if (Front.half_left_dist >= ((MIN_GAP_DEPTH + MAX_GAP_DEPTH)/2.0) )
                vel_msg.angular.z = angularControlP(MIN_DIST, MAX_DIST, ANGULAR_SPEED);
            //else
                // set steering angle to 0 if first corner is close
                //vel_msg.angular.z = 0;
            cmd_vel_pub.publish(vel_msg);

            // check if in range to detect first corner start
            if (Front.left_dist >= MIN_GAP_DEPTH && Front.left_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("Front.left_dist is in range for first corner detection");
                cout << "Front.left_dist" << Front.left_dist << endl;

                cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

                state = FIRST_CORNER_END;
                ROS_INFO("state: FIRST_CORNER_END");
            }
            break;
        case FIRST_CORNER_END:
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = 0;
            cmd_vel_pub.publish(vel_msg);

            // check if in range to detect first corner end
            if (Front.left_dist >= MAX_GAP_DEPTH) {
                ROS_INFO("Front.left_dist is in range for first corner end");
                cout << "Front.left_dist" << Front.left_dist << endl;

                // capture imu yaw here
                R.yaw = yaw;
                ROS_INFO("Captured yaw from IMU");
                
                cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

                state = SECOND_CORNER_START;
                ROS_INFO("state: SECOND_CORNER_START");
            }
            break;
        case SECOND_CORNER_START:
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = angularControlP(MIN_DIST, MAX_DIST, ANGULAR_SPEED);
            cmd_vel_pub.publish(vel_msg);

            if (Front.left_dist >= MIN_GAP_DEPTH && Front.left_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("Front.left_dist is in range for second corner start");
                cout << "Front.left_dist" << Front.left_dist << endl;

                cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

                state = SECOND_CORNER_END;
                ROS_INFO("state: SECOND_CORNER_END");
            }
            break;

        case SECOND_CORNER_END:
            // steer robot
            vel_msg.linear.x = LINEAR_SPEED;
            vel_msg.angular.z = - 0.2 * STEERING_FIRST;
            cmd_vel_pub.publish(vel_msg);

            if (Back.left_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("Back.left_dist is in range for second corner end");
                cout << "Back.left_dist" << Back.left_dist << endl;

                cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

                state = START_PARKING;
                ROS_INFO("state: START_PARKING");
                ROS_INFO("Driving backwards with min steering angle...");
            }
            break;

        case START_PARKING:

            vel_msg.linear.x = - BACKWARD_SPEED_1;
            vel_msg.angular.z = - STEERING_FIRST;
            cmd_vel_pub.publish(vel_msg);

            // check if first backward distance or yaw threshold reached for vehicle rotation limit
            if (Back.middle_dist <= BACKWARD_THRESHOLD_1 || (abs((yaw - R.yaw)) >= YAW_THRESHOLD)) {
                if (abs((yaw - R.yaw)) >= YAW_THRESHOLD) {
                    ROS_INFO("Reached IMU YAW_THRESHOLD");
                    cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;
                }

                ROS_INFO("Back.middle_dist is in range for steering reversal point");
                cout << "Back.middle_dist" << Back.middle_dist << endl;

                cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

                state = PARKING_STATE_1;
                ROS_INFO("state: PARKING_STATE_1");
                ROS_INFO("Steering reversal point reached!");
                ROS_INFO("Driving Backwards with max steering angle...");
            }
            break;

        case PARKING_STATE_1:

            vel_msg.linear.x = - BACKWARD_SPEED_2;
            vel_msg.angular.z = MAX_STEERING;
            cmd_vel_pub.publish(vel_msg);

            // check if second backward distance reached
            if (Back.half_left_dist <= BACKWARD_THRESHOLD_2) {
                ROS_INFO("Back.middle_dist is in range for backward minimum position");
                cout << "Back.middle_dist" << Back.middle_dist << endl;

                cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

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

            if (Front.half_left_dist >= FORWARD_THRESHOLD_2) {
                // ROS_INFO("Front.middle_dist is large enough to do a correction step");
                // cout << "Front.middle_dist" << Front.middle_dist << endl;

                vel_msg.linear.x = LINEAR_SPEED;
                vel_msg.angular.z = 0.3;
                cmd_vel_pub.publish(vel_msg);
            }

            // check if forward min distance reached
            if (Front.half_left_dist <= FORWARD_THRESHOLD_1) {
                // ROS_INFO("Front.middle_dist is to small for correction step");
                // cout << "Front.middle_dist" << Front.middle_dist << endl;

                state = END_PARKING;
                ROS_INFO("state: END_PARKING");
            }

            break;

        case END_PARKING:
            if (display_once) {
                ROS_INFO("Parking Finished. Setting speed to zero... ");
                display_once = false;
            }

            cout << "yaw: " << yaw << " R.yaw" << R.yaw << endl;

            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            cmd_vel_pub.publish(vel_msg);

            // yaw debug section --------
            char input;
            cout << endl << "Do you want to print the yaw vector and save it to a file? Press 'y'" << endl;
            cin >> input;
            if (input == 'y') {
                ofstream myfile;

                string fname = "/home/tas_group_04/" + get_date() + "_yaw.txt"; // create filename using current time and date
		
                myfile.open(fname.c_str());
		cout << "Writing yaw to file: " << fname << endl;
                for (int i=0; i<yaw_vec.size(); i++) {
                    cout << yaw_vec[i] << endl;
                    myfile << yaw_vec[i] << endl;
                }
                myfile.close();
            }
            // ------------

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
