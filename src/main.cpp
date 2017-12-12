#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <forcecontrol/utilities.h>
#include <Eigen/Geometry>

#include <iostream>

#define PI 3.1415926

using namespace std;
using namespace Eigen;

Quaternionf quatMTimes(const Quaternionf &q1, const Quaternionf &q2)
{

    Vector3f v1(q1.x(), q1.y(), q1.z());
    Vector3f v2(q2.x(), q2.y(), q2.z());

    Vector3f cr = v1.cross(v2);

    Quaternionf q;
    q.w() = q1.w()*q2.w() - v1.dot(v2);
    q.x() = v2(0)*q1.w() + q2.w()*v1(0) + cr(0);
    q.y() = v2(1)*q1.w() + q2.w()*v1(1) + cr(1);
    q.z() = v2(2)*q1.w() + q2.w()*v1(2) + cr(2);

    return q;
}

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
    float main_setpos_z;
    float main_setforce_z;
    hd.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    hd.param(std::string("/main_duration"), main_duration, 2.0);
    hd.param(std::string("/main_setpos_z"), main_setpos_z, float(330));
    hd.param(std::string("/main_setforce_z"), main_setforce_z, float(0));
    if (!hd.hasParam("/main_loop_rate"))
      ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << main_loop_rate);
    if (!hd.hasParam("/main_duration"))
      ROS_WARN_STREAM("Parameter [/main_duration] not found, using default: " << main_duration);
    if (!hd.hasParam("/main_setpos_z"))
      ROS_ERROR_STREAM("Parameter [/main_setpos_z] not found!!!");
    if (!hd.hasParam("/main_setforce_z"))
      ROS_WARN_STREAM("Parameter [/main_setforce_z] not found, using default: " << main_setforce_z);

    int Nsteps = int(main_duration*main_loop_rate);
    ros::Rate pub_rate(main_loop_rate);

    // Let the robot go back to origin.
    ROS_INFO_STREAM("[MAIN] Duration: " << main_duration << "sec. " << Nsteps << " steps." << endl);
    // ROS_INFO_STREAM("[MAIN] Press ENTER to begin.\n");
    // getchar();
    
    float pose[7], wrench[6], z0;
    robot.getPose(pose);
    z0 = pose[2];
    Quaternionf q0(pose[3], pose[4], pose[5], pose[6]);
    // Quaternionf qr(1, 0, 0, 0);
    Quaternionf qset(1, 0, 0, 0);
    AngleAxisf aa(0, Vector3f::UnitZ());

    float force[6] = {0,0,0,0,0,0};
    force[2] = main_setforce_z;

    // // 
    // cout << "[Note] This is a temporary version of Main. Are you sure you want to continue?" << endl;
    // cout << "[Note] This is a temporary version of Main. Are you sure you want to continue?" << endl;
    // cout << "[Note] This is a temporary version of Main. Are you sure you want to continue?" << endl;
    // getchar();
    // cout << "Ok. Pauing 3s before motion:" << endl;
    // ros::Duration(3.0).sleep();

    // float originz = pose[2];
    // pose[2] = main_setpos_z;
    // robot.egm->SetCartesian(pose);
    // cout << "Sent! Wait 3s to recover.." << endl;
    // ros::Duration(3.0).sleep();

    // pose[2] = originz;
    // robot.egm->SetCartesian(pose);
    // cout << "Sent! Wait 3s to finish.." << endl;
    // ros::Duration(3.0).sleep();

    // return 0;



    for (int i = 0; i < Nsteps; ++i)
    {
    	ros::Time time_now = ros::Time::now();
    	ros::Duration period(EGM_PERIOD);
        
        if(!robot.getWrench(wrench))
        {
            cout << "[MAIN] force is above safety threhold!" << endl;
            UT::stream_array_in(cout, wrench, 6);
            robot.liftup(float(20));
            ros::Duration(1).sleep();

            break;
        }
        // update
        controller.update(time_now, period);

        // // set z motion
        // float exe_time = float(5*main_loop_rate);
        // if (i < exe_time)
        //     pose[2] = (z0*(exe_time - i) + main_setpos_z*i)/exe_time;
        // else
        // {
        //     pose[2] = main_setpos_z;
        //     controller.setForce(force);

        //     // set rotation
        //     aa.angle() = 0.5*sin(float((i-int(exe_time))%(2*main_loop_rate))/float(2*main_loop_rate)*2*PI);
        //     Quaternionf qr(aa);
        //     // robot.getPose(pose);
        //     qset = quatMTimes(qr, q0);
        //     pose[3] = qset.w();
        //     pose[4] = qset.x();
        //     pose[5] = qset.y();
        //     pose[6] = qset.z();
        // }

        // controller.setPose(pose);




        pub_rate.sleep();
    }

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
