#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>

#include <controller_manager/controller_manager.h>

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Force control starting");
    ros::init(argc, argv, "forcecontrol_node");
    ros::NodeHandle nh;

    ForceControlHardware robot;
    controller_manager::ControllerManager cm(&robot, nh);

    while (true)
    {
    	ros::Time time_now = ros::Time::now();
    	ros::Duration period(EGM_PERIOD);
    	cm.update(time_now, period);
    }
    return 0;
}
