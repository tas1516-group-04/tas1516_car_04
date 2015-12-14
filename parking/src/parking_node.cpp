#include "parking_node.h"
//#include "laserhandle.h"
#include "laserscantopointcloud.h"
#include "features.h"
#include "ScanFeatures.h"

#define MAX_DIST 1.0
#define MIN_DIST 0.3
#define NUM_MEAN_SAMPLES 10

#define MIN_GAP_DEPTH 0.2
#define MAX_GAP_DEPTH 0.35

#define MIN_GAP_LENGTH 0.4
#define MAX_GAP_LENGTH 1.0

#define RANGE_THRESHOLD 0.05


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

    while(1)
    {

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
                state = FIRST_CORNER_START;
            }
            break;
        case FIRST_CORNER_START:
            // check if in range to detect first corner start
            if (Front.diff_dist >= MIN_GAP_DEPTH && Front.diff_dist <= MAX_GAP_DEPTH) {
                ROS_INFO("dist_diff is in range for first corner detection");

                // capture robot position now for first corner start
                C.first.start.xpos = R.xpos;
                C.first.start.ypos = Front.new_dist;

                state = FIRST_CORNER_END;
            }
            break;
        case FIRST_CORNER_END:
            // check if in range to detect first corner end
            if (Front.diff_dist >= RANGE_THRESHOLD) {
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

                // capture robot position now for first corner end
                C.first.end.xpos = R.xpos;
                C.first.end.ypos = Front.new_dist;

                state = SECOND_CORNER_END;
            }
            break;
        case SECOND_CORNER_END:
            if (Front.diff_dist >= RANGE_THRESHOLD) {
                ROS_INFO("dist_diff is in range for first corner end");

                // capture robot position now for first corner end
                C.first.end.xpos = R.xpos;
                C.first.end.ypos = Front.new_dist;

                state = START_PARKING;
            }
            break;
        }

        ros::spin();
    }




    return 0;
}
