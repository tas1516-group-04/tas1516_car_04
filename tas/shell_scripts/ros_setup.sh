#!/bin/sh

echo "This will set up the correct environment to work with the tas-car..."
echo "Please type in the root password:"

#Add source command to bashrc
echo "------------------------------------"
echo "Set up ROS environment"

echo "" >> ~/.bashrc
echo "#Source commands to set up ROS environment" >> ~/.bashrc
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "if [ -f ~/catkin_ws/devel/setup.bash ]; then" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "fi" >> ~/.bashrc


echo "------------------------------------"
echo "ATTENTION: Changes done. Please reboot the system to complete the setup..."


exit

#Export variables to use RVIZ on another computer
#echo "Export variables for external RVIZ"
#export ROS_HOSTNAME=vettel
#export ROS_MASTER_URI=http://vettel:11311


