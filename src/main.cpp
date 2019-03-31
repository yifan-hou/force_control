#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>
#include <forcecontrol/utilities.h>

#include <iostream>

#define PI 3.1415926

using namespace std;
using namespace UT;


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
    controller.reset();

    int main_loop_rate;
    double main_duration;
    float main_setpose[7];
    float main_setforce[6];
    hd.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    hd.param(std::string("/main_duration"), main_duration, 2.0);

    hd.param(std::string("/main_setforce/f1"), main_setforce[0], 0.0f);
    hd.param(std::string("/main_setforce/f2"), main_setforce[1], 0.0f);
    hd.param(std::string("/main_setforce/f3"), main_setforce[2], 0.0f);
    hd.param(std::string("/main_setforce/f4"), main_setforce[3], 0.0f);
    hd.param(std::string("/main_setforce/f5"), main_setforce[4], 0.0f);
    hd.param(std::string("/main_setforce/f6"), main_setforce[5], 0.0f);

    int Nsteps = int(main_duration*main_loop_rate);
    ros::Rate pub_rate(main_loop_rate);

    // Let the robot go back to origin.
    ROS_INFO_STREAM("[MAIN] Duration: " << main_duration << "sec. " << Nsteps << " steps." << endl);
    // ROS_INFO_STREAM("[MAIN] Press ENTER to begin.\n");
    // getchar();

    robot.getPose(main_setpose);

    Matrix6f T0;
    T0 = Matrix6f::Identity();

    controller.reset();
    controller.updateAxis(T0, 3); // all force
    controller.setPose(main_setpose);
    controller.setForce(main_setforce);

    cout << "Main loop begins. " << endl;
    ros::Duration period(EGM_PERIOD);
    for (int i = 0; i < Nsteps; ++i)
    {
        // if (i == main_loop_rate*3)
        // {
        //     // controller.updateAxis(T0, 1);
        //     // main_setpose[0] += xyz_set_diff1(0);
        //     // main_setpose[1] += xyz_set_diff1(1);
        //     // main_setpose[2] += xyz_set_diff1(2);
        //     controller.setPose(main_setpose);

        //     main_setforce[0] = 5;

        //     controller.setForce(main_setforce);
        //     // cout << "update Axis to set 1" << endl;
        // }
        // else if (i == main_loop_rate*10)
        // {
        //     controller.updateAxis(T1, 1);
        //     cout << "update Axis to set 0" << endl;
        // }
        // else if (i == main_loop_rate*15)
        // {
        //     controller.updateAxis(force_selection1);
        //     cout << "update Axis to set 1" << endl;
        // }

        // update
        // cout << "Update for time " << i << " of " << Nsteps << "." << endl;
        ros::Time time_now = ros::Time::now();
        controller.update(time_now, period);


        // // check force feedback direction
        // float w[6] = {0}; // tool frame
        // float p[7] = {0};
        // robot.getWrench(w);
        // robot.getPose(p);

        // Quaternionf qn;
        // qn.w() = p[3];
        // qn.x() = p[4];
        // qn.y() = p[5];
        // qn.z() = p[6];
        // Eigen::Vector3f f_T;
        // f_T << w[0], w[1], w[2];
        // Eigen::Vector3f f_W = qn._transformVector(f_T);

        // cout << "time " << i << ", f_W: "
        //         << f_W[0] << "|"
        //         << f_W[1] << "|"
        //         << f_W[2] << "f_T: "
        //         << f_T[0] << "|"
        //         << f_T[1] << "|"
        //         << f_T[2] << "q:"
        //         << p[3] << "|"
        //         << p[4] << "|"
        //         << p[5] << "|"
        //         << p[6] << endl;


        pub_rate.sleep();
    }

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
