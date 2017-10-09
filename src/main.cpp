#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <yifanlibrary/TimerLinux.h>

#include <iostream>


using namespace std;

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Force control starting");
    ros::init(argc, argv, "forcecontrol_node");
    ros::NodeHandle hd;

    Timer The_timer; // the timer that is used by all threads

    ForceControlHardware robot;
    ForceControlController controller;

    The_timer.tic(); // start ticking
    
    robot.init(hd, &The_timer); // robot must be initialized before controller
    controller.init(hd, &robot, &The_timer);
    
    int main_loop_rate;
    double main_duration;
    hd.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    hd.param(std::string("/main_duration"), main_duration, 2.0);
    if (!hd.hasParam("/main_loop_rate"))
      ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << main_loop_rate);
    if (!hd.hasParam("/main_duration"))
      ROS_WARN_STREAM("Parameter [/main_duration] not found, using default: " << main_duration);

    int Nsteps = int(main_duration*main_loop_rate);
    ros::Rate pub_rate(main_loop_rate);

    // Let the robot go back to origin.
    ROS_INFO_STREAM("[MAIN] Duration: " << main_duration << "sec. " << Nsteps << " steps." << endl);
    ROS_INFO_STREAM("[MAIN] Press ENTER to begin.\n");
    getchar();
    

    for (int i = 0; i < Nsteps; ++i)
    {
    	ros::Time time_now = ros::Time::now();
    	ros::Duration period(EGM_PERIOD);
        controller.update(time_now, period);

        pub_rate.sleep();
    }

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
