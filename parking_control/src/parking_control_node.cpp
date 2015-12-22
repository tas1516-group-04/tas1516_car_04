#include "control/control.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);

    int display_counter = 0;

    while(ros::ok())
    {
        display_counter++;

        if(autonomous_control.control_Mode.data==0)
        {
            if (display_counter%(50*2) == 0)
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
                if (display_counter%(50*2) == 0)
                    ROS_INFO("Automatic Control!");

                if(autonomous_control.cmd_linearVelocity>0)
                {
                    autonomous_control.control_servo.x = 1500 + autonomous_control.cmd_linearVelocity * 100;
                }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                    autonomous_control.control_servo.x = 1500 - autonomous_control.cmd_linearVelocity * 100;
                }
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }
                cout << "Speed: " << autonomous_control.control_servo.x << endl;
                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
                cout << "Steering Angle: " << autonomous_control.control_servo.y << endl;
            }
            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
