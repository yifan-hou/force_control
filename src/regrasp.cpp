#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <forcecontrol/utilities.h>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <unistd.h>

#include "ZZJHand.h"

#define PI 3.1415926

using namespace std;
using namespace Eigen;


Quaternionf quatMTimes(Quaternionf q1, Quaternionf q2)
{
    float s1 = q1.w();
    Vector3f v1(q1.x(), q1.y(), q1.z());

    float s2 = q2.w();
    Vector3f v2(q2.x(), q2.y(), q2.z());

    float cr_v1 = v1(1)*v2(2) - v1(2)*v2(1);
    float cr_v2 = v1(2)*v2(0) - v1(0)*v2(2);
    float cr_v3 = v1(0)*v2(1) - v1(1)*v2(0);

    Quaternionf qp;
    qp.w() = s1*s2 - v2.dot(v1);
    qp.x() = v2(0)*s1 + s2*v1(0) + cr_v1;
    qp.y() = v2(1)*s1 + s2*v1(1) + cr_v2;
    qp.z() = v2(2)*s1 + s2*v1(2) + cr_v3;

    return qp;
}


float angBTquat(Quaternionf q1, Quaternionf q2)
{
    q1.normalize();
    q2.normalize();

    Quaternionf q_ = quatMTimes(q1.inverse(), q2);

    float ang = 2*acos(q_.w()); // acos: [0, pi]

    if (ang > PI){
        ang = ang - 2*PI;
    }

    return fabs(ang);
}

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Regrasping Experiment is starting");
    ros::init(argc, argv, "regrasping_node");
    ros::NodeHandle hd;

    // --------------------------------------------------------
    // Read Trajectory
    // --------------------------------------------------------

    ifstream f_pgrp, f_qgrp, f_rtype, f_N;
    f_pgrp.open("/usr0/home/yifanh/Git/regrasp3d/results/pgrp.txt");
    f_qgrp.open("/usr0/home/yifanh/Git/regrasp3d/results/qgrp.txt");
    f_rtype.open("/usr0/home/yifanh/Git/regrasp3d/results/rtype.txt");
    f_N.open("/usr0/home/yifanh/Git/regrasp3d/results/N.txt");

    if (!f_pgrp) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/pgrp.txt'.";
        exit(1); // terminate with error
    }
    if (!f_qgrp) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/qgrp.txt'.";
        exit(1); // terminate with error
    }
    if (!f_rtype) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/rtype.txt'.";
        exit(1); // terminate with error
    }
    if (!f_N) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/N.txt'.";
        exit(1); // terminate with error
    }

    int N_TRJ;
    f_N >> N_TRJ;
    cout << "N_TRJ: " << N_TRJ << endl;
    getchar();

    MatrixXf pgrp(3, N_TRJ);
    MatrixXf qgrp(4, N_TRJ);
    bool rtype[N_TRJ];

    for (int i = 0; i < N_TRJ; ++i)
    {
        f_qgrp >> qgrp(0, i) >> qgrp(1, i) >> qgrp(2, i) >> qgrp(3, i);
        f_pgrp >> pgrp(0, i) >> pgrp(1, i) >> pgrp(2, i);

        f_rtype >> rtype[i];

        cout << i << " rtype: " << rtype[i];
        cout << " qgrp: " << qgrp(0, i) << " " << qgrp(1, i) << " " << qgrp(2, i) << " " << qgrp(3, i);
        cout << " pgrp: " << pgrp(0, i) << " " << pgrp(1, i) << " " << pgrp(2, i) << endl;
    }


    // initialize gripper
    ZZJHand *hand = ZZJHand::Instance();
    hand->openEpos();
    hand->getHomePos();

    cout << endl;
    cout << "Press ENTER to continue.";
    getchar();

    // --------------------------------------------------------
    // Move the robot to grasp preparation pose
    // --------------------------------------------------------
    // initialization
    int main_loop_rate;
    hd.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    if (!hd.hasParam("/main_loop_rate"))
        ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << main_loop_rate);
    ros::Rate pub_rate(main_loop_rate);

    ForceControlHardware robot;
    ForceControlController controller;

    std::chrono::high_resolution_clock::time_point TheTime0;
    TheTime0 = std::chrono::high_resolution_clock::now();

    robot.init(hd, TheTime0); // robot must be initialized before controller
    controller.init(hd, &robot, TheTime0);
    
    
    float pose_set[7], pose[7];
    pose_set[0] = pgrp(0,0);
    pose_set[1] = pgrp(1,0) + 100;
    pose_set[2] = pgrp(2,0) + 0;
    pose_set[3] = qgrp(0,0);
    pose_set[4] = qgrp(1,0);
    pose_set[5] = qgrp(2,0);
    pose_set[6] = qgrp(3,0);
    robot.egm->SetCartesian(pose_set);


    cout << "Initial pose sent. Waiting for converging..." << endl;


    Quaternionf q, qset;
    Vector3f p, pset;
    pset(0)  = pose_set[0];
    pset(1)  = pose_set[1];
    pset(2)  = pose_set[2];
    qset.w() = pose_set[3];
    qset.x() = pose_set[4];
    qset.y() = pose_set[5];
    qset.z() = pose_set[6];
    for (int i = 0; i < 5000; ++i)
    {
        robot.getPose(pose);
        p(0)  = pose[0];
        p(1)  = pose[1];
        p(2)  = pose[2];
        q.w() = pose[3];
        q.x() = pose[4];
        q.y() = pose[5];
        q.z() = pose[6];

        if( ((pset-p).norm() < 0.1) && (angBTquat(q, qset) < 1e-3) )
            break;

        pub_rate.sleep();
    }

    unsigned milliseconds = 500;
    usleep(milliseconds * 1000);

    // grasp
    if(hand->DoFirmGrasp() != MMC_SUCCESS)
    {
        hand->closeEpos();
        return -1;
    } 

    // --------------------------------------------------------
    //  Main loop
    // --------------------------------------------------------
    ros::Time time_now = ros::Time::now();
    ros::Duration period(EGM_PERIOD);
    for (int fr = 1; fr < N_TRJ; ++fr)
    {
        cout << "Running frame number " << fr << endl;
        if ((fr==1) || (rtype[fr-1]!=rtype[fr]) )
        {
            if (rtype[fr] == true)
            {
                if(hand->DoPivotGrasp() != MMC_SUCCESS)
                {
                    hand->closeEpos();
                    return -1;
                } 
            }
            else
            {
                if(hand->DoFirmGrasp() != MMC_SUCCESS)
                {
                    hand->closeEpos();
                    return -1;
                } 
            }
        }

        pose_set[0] = pgrp(0,fr);
        pose_set[1] = pgrp(1,fr) + 100;
        pose_set[2] = pgrp(2,fr) + 0;
        pose_set[3] = qgrp(0,fr);
        pose_set[4] = qgrp(1,fr);
        pose_set[5] = qgrp(2,fr);
        pose_set[6] = qgrp(3,fr);

        // robot.egm->SetCartesian(pose_set);

        controller.setPose(pose_set);
        controller.update(time_now, period);
        pub_rate.sleep();
    }


    hand->closeEpos();

    // robot.getPose(pose);
    // z0 = pose[2];
    // Quaternionf q0(pose[3], pose[4], pose[5], pose[6]);
    // // Quaternionf qr(1, 0, 0, 0);
    // Quaternionf qset(1, 0, 0, 0);
    // AngleAxisf aa(0, Vector3f::UnitZ());

    // float force[6] = {0,0,0,0,0,0};
    // force[2] = main_setforce_z;

    // for (int i = 0; i < Nsteps; ++i)
    // {
    // 	ros::Time time_now = ros::Time::now();
    // 	ros::Duration period(EGM_PERIOD);
        
    //     if(!robot.getWrench(wrench))
    //     {
    //         cout << "[MAIN] force is above safety threhold!" << endl;
    //         UT::stream_array_in(cout, wrench, 6);
    //         robot.liftup(float(20));
    //         ros::Duration(1).sleep();

    //         break;
    //     }
    //     // update
    //     controller.update(time_now, period);

    //     // // set z motion
    //     // float exe_time = float(5*main_loop_rate);
    //     // if (i < exe_time)
    //     //     pose[2] = (z0*(exe_time - i) + main_setpos_z*i)/exe_time;
    //     // else
    //     // {
    //     //     pose[2] = main_setpos_z;
    //     //     controller.setForce(force);

    //     //     // set rotation
    //     //     aa.angle() = 0.5*sin(float((i-int(exe_time))%(2*main_loop_rate))/float(2*main_loop_rate)*2*PI);
    //     //     Quaternionf qr(aa);
    //     //     // robot.getPose(pose);
    //     //     qset = quatMTimes(qr, q0);
    //     //     pose[3] = qset.w();
    //     //     pose[4] = qset.x();
    //     //     pose[5] = qset.y();
    //     //     pose[6] = qset.z();
    //     // }

    //     // controller.setPose(pose);




    //     pub_rate.sleep();
    // }

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
