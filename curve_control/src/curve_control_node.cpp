#include "control/control.h"
#include <nav_msgs/Path.h>

// get transform to obtain position of robot on global path
// necessary, does part of global path vanish, when reached by robot -->
// can position be obtained from global path???
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define TOLERANCE           0.5
#define LOOK_AHEAD_DIST     2.0
#define LOOK_AHEAD_POSES    20
#define WEIGHTING_FACTOR    1.0
#define DELTA_POSES         5

#define MAX_SPEED           3.0

// TODO safe to write to global variable
nav_msgs::Path::ConstPtr global_path;

void navPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    int numposes = msg.get()->poses.size();
    for (int i=0; i < numposes; i++) {
        std::cout << msg.get()->poses[i];
    }
    // TODO check if called once or often
    global_path = msg;

}

void poseCallback(const nav_msgs::Path::ConstPtr& msg)
{
    int numposes = msg.get()->poses.size();
    for (int i=0; i < numposes; i++) {
        std::cout << msg.get()->poses[i];
    }
}


/* calc derivative of global path at position x
 * path need to be rotated in base link frame before passed to the function
 */
/*
float calcFirstDerivative(int x, int deltaXPoses, const nav_msgs::Path::ConstPtr& msg)
{
    float derivative = (msg.get()->poses[x][0] - msg.get()->poses[lookAheadPoses + deltaXPoses])
            / deltaXPoses;

    return derivative;
}

float calcFirstDerivative(float deltaX, float y1, float y2)
{
    return (y2 - y1) / deltaX;
}
*/

void calcSpeed()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "curve_control");
    control autonomous_control;

    ros::NodeHandle n;
    // TODO check topic for path
    ros::Subscriber path_sub = n.subscribe("/move_base/TrajectoryPlanner/Path", 1000, navPathCallback);
    // TODO check topic for pose
    // ros::Subscriber pose_sub = n.subscribe("/tf", 1000, poseCallback);

    tf::TransformListener tf_listener;
    tf::Transform transform;


    ros::Rate loop_rate(50);

    while(ros::ok())
    {

        tf::Vector3 origin = transform.getOrigin();

        float speed = autonomous_control.cmd_linearVelocity;

        double alpha = tf::getYaw(transform.getRotation());

        // calc distance between current pos and look ahead of global path
        // Check if nav goal is reached and LOOK_AHEAD_POSES are out of bounds then
        float distX = origin[0] - global_path.get()->poses[LOOK_AHEAD_POSES].pose.position.x;
        float distY = origin[1] - global_path.get()->poses[LOOK_AHEAD_POSES].pose.position.y;

        float aheadX = global_path.get()->poses[LOOK_AHEAD_POSES].pose.position.x;
        float aheadXdiff = global_path.get()->poses[LOOK_AHEAD_POSES + DELTA_POSES].pose.position.x;
        float aheadY = global_path.get()->poses[LOOK_AHEAD_POSES].pose.position.y;
        float aheadYdiff = global_path.get()->poses[LOOK_AHEAD_POSES + DELTA_POSES].pose.position.y;

        // rotate global path points into car coordinate system, offset is of no interest for derivative
        aheadX = cos(alpha)*aheadX - sin(alpha)*aheadX;
        aheadY = sin(alpha)*aheadY + cos(alpha)*aheadY;
        aheadXdiff = cos(alpha)*aheadXdiff - sin(alpha)*aheadXdiff;
        aheadYdiff = cos(alpha)*aheadYdiff - sin(alpha)*aheadYdiff;

        // calc derivative
        float deltaX = aheadXdiff - aheadX;
        float deltaY = aheadYdiff - aheadY;
        float derivative = deltaY / deltaX;

        // make always positive
        derivative = abs(derivative);

        /* calc speed based on derivative of path look ahead
         * big derivative = fast decelleration
         * small derivative = slow decelleration
         * derivative small = acceleration
         */
        if (derivative >=0 || derivative < 10) {
            speed += WEIGHTING_FACTOR * derivative;
        }
        else if (derivative >= 10) {
            speed -= WEIGHTING_FACTOR * derivative;
        }
        if (speed >= MAX_SPEED) {
            speed = MAX_SPEED;
        }

        /*
         *  y
         *
         *  ^
         *  |
         *  |
         *  |
         *  |             p'
         *  |           p
         *  |
         *  |_____________________________________ >  x
         *
         */

        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
        }
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                ROS_INFO("Automatic Control!");
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    autonomous_control.control_servo.x = 1550;
                }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                    autonomous_control.control_servo.x = 1300;
                }
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
