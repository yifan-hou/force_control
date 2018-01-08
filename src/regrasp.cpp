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

Vector2f goTowards(const Vector2f from, const Vector2f to, const float delta)
{
    Vector2f diff = to - from;
    float length  = diff.norm();
    if (length < delta)
    {
        return to;
    }
    else
    {
        diff.normalize();
        return from + diff*delta;
    }
}


int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Regrasping Experiment is starting");
    ros::init(argc, argv, "regrasping_node");
    ros::NodeHandle hd;

    // --------------------------------------------------------
    // Read Trajectory
    // --------------------------------------------------------

    ifstream f_N, f_rtype, f_stuck, f_qgrp, f_grp0, f_grpz, f_grpxy_delta;
    f_N.open("/usr0/home/yifanh/Git/regrasp3d/results/N.txt");
    f_rtype.open("/usr0/home/yifanh/Git/regrasp3d/results/rtype.txt");
    f_stuck.open("/usr0/home/yifanh/Git/regrasp3d/results/stuck.txt");
    f_qgrp.open("/usr0/home/yifanh/Git/regrasp3d/results/qgrp.txt");
    f_grp0.open("/usr0/home/yifanh/Git/regrasp3d/results/grp0.txt");
    f_grpz.open("/usr0/home/yifanh/Git/regrasp3d/results/grpz.txt");
    f_grpxy_delta.open("/usr0/home/yifanh/Git/regrasp3d/results/grpxy_delta.txt");

    if (!f_N) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/N.txt'.";
        exit(1); // terminate with error
    }
    if (!f_rtype) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/rtype.txt'.";
        exit(1); // terminate with error
    }
    if (!f_stuck) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/stuck.txt'.";
        exit(1); // terminate with error
    }
    if (!f_qgrp) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/qgrp.txt'.";
        exit(1); // terminate with error
    }
    if (!f_grp0) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/grp0.txt'.";
        exit(1); // terminate with error
    }
    if (!f_grpz) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/grpz.txt'.";
        exit(1); // terminate with error
    }
    if (!f_grpxy_delta) 
    {
        cerr << "Unable to open file at '/usr0/home/yifanh/Git/regrasp3d/results/grpxy_delta.txt'.";
        exit(1); // terminate with error
    }

    int N_TRJ;
    f_N >> N_TRJ;
    cout << "N_TRJ: " << N_TRJ << endl;
    getchar();

    bool rtype[N_TRJ];
    bool stuck[N_TRJ];
    MatrixXf qgrp(4, N_TRJ);
    Vector3f grp0;
    float grpz[N_TRJ];
    MatrixXf grpxy_delta(2, N_TRJ);

    f_grp0 >> grp0(0) >> grp0(1) >> grp0(2);
    for (int i = 0; i < N_TRJ; ++i)
    {
        f_rtype >> rtype[i];
        f_stuck >> stuck[i];
        f_qgrp >> qgrp(0, i) >> qgrp(1, i) >> qgrp(2, i) >> qgrp(3, i);
        f_grpz >> grpz[i];
        f_grpxy_delta >> grpxy_delta(0, i) >> grpxy_delta(1, i);

        cout << i << " rtype: " << rtype[i] << " stuck: " << stuck[i];
        cout << " qgrp: " << qgrp(0, i) << " " << qgrp(1, i) << " " << qgrp(2, i) << " " << qgrp(3, i);
        cout << " grpz: " << grpz[i] << endl;
    }


    // initialize gripper
    ZZJHand *hand = ZZJHand::Instance();
    hand->openEpos();
    hand->getHomePos();

    cout << endl;
    cout << "Press ENTER to continue.";
    getchar();

    // initialization
    int main_loop_rate;
    hd.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    if (!hd.hasParam("/main_loop_rate"))
        ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << main_loop_rate);
    ros::Rate pub_rate(main_loop_rate);


    const float DELTA_XY = 20.0/float(main_loop_rate); 

    ForceControlHardware robot;
    ForceControlController controller;

    std::chrono::high_resolution_clock::time_point TheTime0;
    TheTime0 = std::chrono::high_resolution_clock::now();

    robot.init(hd, TheTime0); // robot must be initialized before controller
    controller.init(hd, &robot, TheTime0);
    
    // --------------------------------------------------------
    // Move to first frame 
    // --------------------------------------------------------
    
    float pose_set[7], pose[7];
    pose_set[0] = grp0(0);
    pose_set[1] = grp0(1);
    pose_set[2] = grp0(2);
    pose_set[3] = qgrp(0,0);
    pose_set[4] = qgrp(1,0);
    pose_set[5] = qgrp(2,0);
    pose_set[6] = qgrp(3,0);
    robot.egm->SetCartesian(pose_set);

    // --------------------------------------------------------
    // Wait until reach the first frame
    // --------------------------------------------------------
    
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

    bool is_stuck = false;
    Vector2f xy;
    for (int fr = 1; fr < N_TRJ; ++fr)
    {
        cout << "Running frame number " << fr << endl;
        // ------------------------
        //  Hand
        // ------------------------
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

        // ------------------------
        // Arm
        // ------------------------
        // read feedback
        robot.getPose(pose);
        // q.w() = pose[3];
        // q.x() = pose[4];
        // q.y() = pose[5];
        // q.z() = pose[6];

        if (stuck[fr] == true)
        {
            if (!is_stuck)
            {
                xy(0)  = pose[0];
                xy(1)  = pose[1];
            }
            xy(0) = xy(0) + grpxy_delta(0, fr); // move an arc   
            xy(1) = xy(1) + grpxy_delta(1, fr);                
            is_stuck = true;
        }
        else
        {
            xy(0)    = pose[0];
            xy(1)    = pose[1];
            xy       = goTowards(xy, grp0.head(2), DELTA_XY);
            is_stuck = false;
        }

        pose_set[0] = xy(0);
        pose_set[1] = xy(1);
        pose_set[2] = grpz[fr];

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



    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
