
#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>

#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>

#include <ati_netft/ati_netft.h>
#include <abb_egm/abb_egm.h>
#include <ur_socket/ur_socket.h>

#define PI 3.1415926

using namespace std;
using namespace RUT;


int main(int argc, char* argv[]) {
    ROS_INFO_STREAM("Force control test node starting");
    ros::init(argc, argv, "forcecontrol_node");
    ros::NodeHandle hd;

    /*  First, we need to instantiate and initialize objects of
     *  ForceControlHardware and ForceControlController.
     */
    Clock::time_point time0 = std::chrono::high_resolution_clock::now();
    ATINetft ati;
    cout << "[test] initializing ft sensor:\n";
    ati.init(hd, time0);
    cout << "[test] initializing robot:\n";
    URSocket *robot = URSocket::Instance();
    robot->init(hd, time0);

    ForceControlHardware hardware;
    ForceControlController controller;
    cout << "[test] initializing hardware:\n";
    hardware.init(hd, time0, &ati, robot);
    cout << "[test] initializing controller:\n";
    controller.init(hd, &hardware, time0);
    cout << "[test] initializing is done:\n";

    /*  Some parameters for the test.
     */
    int main_loop_rate;
    double main_duration;
    hd.param(std::string("/forcecontrol/main_loop_rate"), main_loop_rate, 500);
    hd.param(std::string("/main_duration"), main_duration, 2.0);
    cout << "[test] initializing is done:\n";

    ros::Rate pub_rate(main_loop_rate);
    int Nsteps = int(main_duration*main_loop_rate);

    ROS_INFO_STREAM("[MAIN] Duration: " << main_duration << "sec. " << Nsteps
            << " steps." << endl);

    /*  Set control commands.
     */
    double setpose[7];
    double setforce[6] = {0};
    hardware.getPose(setpose);
    cout << "[test] Press Enter to move: \n";
    getchar();
    double pose_fb[7];
    for (int i = 0; i < 200; ++i) {
        setpose[0] += 0.2;
        hardware.setPose(setpose);
        usleep(2*1000);
        hardware.getPose(pose_fb);
        cout <<"i: " << i << ", x: " << setpose[0] << ", x_fb: " << pose_fb[0] << endl;
    }
    for (int i = 0; i < 200; ++i) {
        setpose[0] -= 0.2;
        hardware.setPose(setpose);
        usleep(2*1000);
        hardware.getPose(pose_fb);
        cout <<"i: " << i << ", x: " << setpose[0] << ", x_fb: " << pose_fb[0] << endl;
    }
    for (int i = 0; i < 200; ++i) {
        hardware.setPose(setpose);
        usleep(2*1000);
        hardware.getPose(pose_fb);
        cout <<"i: " << i << ", x: " << setpose[0] << ", x_fb: " << pose_fb[0] << endl;
    }

    cout << "[test] Press Enter to begin force control: \n";
    getchar();

    Matrix6d T0, T1, T2;
    T0 = Matrix6d::Identity();
    T1 << 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1,
          1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0;
    T1 << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1,
          0, 0, 1, 0, 0, 0;
    /*  Set controls.
     *  Note that calling reset() right after init() is not necessary;
     *  This is showing the general procedure for setting controls.
     */
    controller.reset();
    controller.updateAxis(T1, 6);
    controller.setPose(setpose); // after setPose, you must call update() before
                                 // calling updateAxis()
    controller.setForce(setforce);

    // ROS_INFO_STREAM("[MAIN] Press ENTER to begin.\n");
    // getchar();
    cout << "Main loop begins. " << endl;
    RUT::Timer timer;
    double time_elapsed = 0;
    for (int i = 0; i < Nsteps; ++i) {
        // if (i == main_loop_rate*10)
        // {
        //     controller.updateAxis(T1, 3);
        //     // setpose[0] += xyz_set_diff1(0);
        //     // setpose[1] += xyz_set_diff1(1);
        //     // setpose[2] += xyz_set_diff1(2);
        //     // controller.setPose(setpose);

        //     // setforce[0] = 5;

        //     // controller.setForce(setforce);
        //     cout << "update Axis to set 1" << endl;
        // }

        // update
        timer.tic();
        ros::Time time_now = ros::Time::now();
        if (!controller.update()) {
            // wrench feedback greater than threshold
            double w_ati[6] = {0}; // tool frame
            ati.getWrenchSensor(w_ati);
            ROS_WARN_STREAM("Not safe!! Wrench: " << w_ati[0] <<
                "|" << w_ati[1] << "|" << w_ati[2] << "|   |" << w_ati[3] <<
                "|" << w_ati[4] << "|" << w_ati[5]);
        }
        time_elapsed = timer.toc();
        cout << "Timestep " << i << " of " << Nsteps << ". Time: "<< time_elapsed << "ms. Command:";
        stream_array_in(cout, controller._pose_sent_to_robot, 7);
        cout << endl;

        // double w_ati[6] = {0}; // tool frame
        // ati.getWrenchSensor(w_ati);
        // cout << "Timestep " << i << " of " << Nsteps << ". Time: "<< time_elapsed << "ms. Force:";
        // cout << w_ati[0] << "|" << w_ati[1] << "|" << w_ati[2] << "|   |" << w_ati[3] <<
        //         "|" << w_ati[4] << "|" << w_ati[5] << endl;


        // // check force feedback direction
        // float p[7] = {0};
        // hardware.getPose(p);

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
