#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <forcecontrol/utilities.h>
#include <Eigen/Geometry>

#include <iostream>

#define PI 3.1415926

using namespace std;
using namespace Eigen;


int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Force control starting");
    ros::init(argc, argv, "forcecontrol_node");
    ros::NodeHandle hd;

    ForceControlHardware robot;
    ForceControlController controller;

    std::chrono::high_resolution_clock::time_point TheTime0;
    TheTime0 = std::chrono::high_resolution_clock::now();
    
    robot.init(hd, TheTime0); // robot must be initialized before controller
    controller.init(hd, &robot, TheTime0);
    
    int main_loop_rate;
    double main_duration;
    float main_setpos[7];
    float main_setforce_z;
    hd.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    hd.param(std::string("/main_duration"), main_duration, 2.0);
    hd.param(std::string("/main_setpos/x"), main_setpos[0], 0.0f);
    hd.param(std::string("/main_setpos/y"), main_setpos[1], 300.0f);
    hd.param(std::string("/main_setpos/z"), main_setpos[2], 435.0f);
    hd.param(std::string("/main_setpos/q1"), main_setpos[3], 0.0f);
    hd.param(std::string("/main_setpos/q2"), main_setpos[4], 0.0f);
    hd.param(std::string("/main_setpos/q3"), main_setpos[5], 1.0f);
    hd.param(std::string("/main_setpos/q4"), main_setpos[6], 0.0f);

    int Nsteps = int(main_duration*main_loop_rate);
    ros::Rate pub_rate(main_loop_rate);

    // Let the robot go back to origin.
    ROS_INFO_STREAM("[MAIN] Duration: " << main_duration << "sec. " << Nsteps << " steps." << endl);
    // ROS_INFO_STREAM("[MAIN] Press ENTER to begin.\n");
    // getchar();
    
    float pose[7], wrench[6], z0;
    // robot.getPose(pose);
    controller.setPose(main_setpos);

    // z0 = pose[2];
    // Quaternionf q0(pose[3], pose[4], pose[5], pose[6]);
    // // Quaternionf qr(1, 0, 0, 0);
    // Quaternionf qset(1, 0, 0, 0);
    // AngleAxisf aa(0, Vector3f::UnitZ());

    float force[6] = {0,0,0,0,0,0};
    controller.setForce(force);

    // bool force_selection0[3]{1, 1, 0};
    // bool force_selection1[3]{0, 0, 1};

    cout << "Main loop begins. " << endl;
	ros::Duration period(EGM_PERIOD);
    for (int i = 0; i < Nsteps; ++i)
    {
        
        // if(!robot.getWrench(wrench))
        // {
        //     cout << "[MAIN] force is above safety threhold!" << endl;
        //     UT::stream_array_in(cout, wrench, 6);
        //     robot.liftup(float(20));
        //     ros::Duration(1).sleep();

        //     break;
        // }

        // cout << "wrench: " ;
        // for (int i = 0; i < 6; ++i)
        // {
        //     cout << wrench[i] << "\t";
        // }
        // cout << endl;

        // if (i == main_loop_rate*5)
        // {
        //     controller.updateAxis(force_selection1);
        //     cout << "update Axis to set 1" << endl;
        // }
        // else if (i == main_loop_rate*10)
        // {
        //     controller.updateAxis(force_selection0);
        //     cout << "update Axis to set 0" << endl;
        // }
        // else if (i == main_loop_rate*15)
        // {
        //     controller.updateAxis(force_selection1);
        //     cout << "update Axis to set 1" << endl;
        // }

        // update
        controller.setPose(main_setpos);
        cout << "Update for time " << i << " of " << Nsteps << "." << endl;
        ros::Time time_now = ros::Time::now();
        controller.update(time_now, period);
        pub_rate.sleep();
    }

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
