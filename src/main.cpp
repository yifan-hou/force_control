#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <yifanlibrary/TimerLinux.h>

#include <iostream>

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Force control starting");
    ros::init(argc, argv, "forcecontrol_node");
    ros::NodeHandle hd;
    ros::NodeHandle phd("~");

    Timer timer; ///< high resolution timer.

    ForceControlHardware robot;
    ForceControlController controller;

    controller.init(&robot);
    robot.init(hd, hd);

    while (true)
    {
    	ros::Time time_now = ros::Time::now();
    	ros::Duration period(EGM_PERIOD);
        controller.update(time_now, period);
    }
    return 0;
}
