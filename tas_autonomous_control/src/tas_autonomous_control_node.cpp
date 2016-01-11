#define MAX_VEL 1600
#define MIN_VEL 1525

#include "control/control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            //ROS_INFO("Manually Control!");
            //std::cout << "vel_cmd: " << autonomous_control.vel_linearVelocity << std::endl;
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
                // ROS_INFO("Automatic Control!");
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    std::cout << "vel_cmd: " << autonomous_control.vel_linearVelocity << std::endl;
                    std::cout << "cmd_vel: " << autonomous_control.cmd_linearVelocity << std::endl;
                    if (autonomous_control.cmd_angularVelocity == 101)
                    {
                        // SpeedController will send -1 when no speed can be calculated.
                        if (autonomous_control.vel_linearVelocity != -1)
                        {
                            // vel_linearVelocity will be between 0 and 1
                            autonomous_control.control_servo.x = 1550 + 10 * autonomous_control.vel_linearVelocity;
                        }
                        else
                        {
                            std::cout << "SpeedController Error -1" << std::endl;
                            autonomous_control.control_servo.x = 1500;
                        }
                        std::cout << "Servo_controller: " << autonomous_control.control_servo.x << std::endl;
                    }
                    else
                    {
                        autonomous_control.control_servo.x = 1550;
                        std::cout << "Servo_planner: " << autonomous_control.control_servo.x << std::endl;
                    }

                    // clip cmd_vel between [0,1]
                    // servo = min + (max-min)*clipped
                    // autonomous_control.control_servo.x = 1525 + 50 * autonomous_control.cmd_linearVelocity;
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
