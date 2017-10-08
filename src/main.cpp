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

    int main_loop_rate;
    hd.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    if (!hd.hasParam("/main_loop_rate"))
      ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << main_loop_rate);

    ros::Rate pub_rate(main_loop_rate);

    Timer The_timer; // the timer that is used by all threads

    ForceControlHardware robot;
    ForceControlController controller;

    The_timer.tic(); // start ticking
    
    controller.init(hd, &robot);
    robot.init(hd, &The_timer);
    
    // Let the robot go back to origin.
    ROS_INFO_STREAM("[MAIN] Press ENTER to move to touching pose:\n");
    getchar();
    
    float pose[7];
    float wrench[6];
    robot.getState(pose, wrench);

    pose[2] = 374.16; // just touching
    robot.setControl(pose);

    ROS_INFO_STREAM("[MAIN] Press ENTER to begin motion:\n");
    getchar();
    

    pose[2] = 370; // pressing
    robot.setControl(pose);
    ros::Duration(0.5).sleep(); 

    pose[2] = 374.16; // back to just touching
    robot.setControl(pose);
    ros::Duration(0.5).sleep(); 


    // for (int i = 0; i < 300; ++i)
    // {
    // 	ros::Time time_now = ros::Time::now();
    // 	ros::Duration period(EGM_PERIOD);
    //     controller.update(time_now, period);

    //     pub_rate.sleep();
    // }

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
