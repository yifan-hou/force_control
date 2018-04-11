#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#include <forcecontrol/utilities.h>
#include <Eigen/Geometry>

#include <iostream>

#define PI 3.1415926

#define Nx 5 
#define Ny 5 

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
    ROS_INFO_STREAM("Calibrating weight of tool");
    ros::init(argc, argv, "forcecontrol_node");
    ros::NodeHandle hd;

    ForceControlHardware robot;

    std::chrono::high_resolution_clock::time_point TheTime0;
    TheTime0 = std::chrono::high_resolution_clock::now();
    
    robot.init(hd, TheTime0); // robot must be initialized before controller
    
    int Nx, Ny;
    float AngX, AngY;
    hd.param(std::string("/calibration/Number_of_tilt_x"), Nx, 3);
    if (!hd.hasParam("/calibration/Number_of_tilt_x"))
      ROS_ERROR_STREAM("Parameter [/calibration/Number_of_tilt_x] not found!!!");

    hd.param(std::string("/calibration/Number_of_tilt_y"), Ny, 3);
    if (!hd.hasParam("/calibration/Number_of_tilt_y"))
      ROS_ERROR_STREAM("Parameter [/calibration/Number_of_tilt_y] not found!!!");

    hd.param(std::string("/calibration/Angle_of_tilt_x"), AngX, 3);
    if (!hd.hasParam("/calibration/Angle_of_tilt_x"))
      ROS_ERROR_STREAM("Parameter [/calibration/Angle_of_tilt_x] not found!!!");

    hd.param(std::string("/calibration/Angle_of_tilt_y"), AngY, 3);
    if (!hd.hasParam("/calibration/Angle_of_tilt_y"))
      ROS_ERROR_STREAM("Parameter [/calibration/Angle_of_tilt_y] not found!!!");

    float *ang_x_array = new float[Nx];
    float *ang_y_array = new float[Ny];

    for (int i = 0; i < Nx; ++i)   ang_x_array[i] = -AngX + 2*AngX*float(i)/float(Nx-1);
    for (int i = 0; i < Ny; ++i)   ang_y_array[i] = -AngY + 2*AngY*float(i)/float(Ny-1);

    float pose[7], wrench[6];
    robot.getPose(pose);
    Quaternionf q0(pose[3], pose[4], pose[5], pose[6]);

    AngleAxisf aaX(0, Vector3f::UnitX());
    AngleAxisf aaY(0, Vector3f::UnitY());
    Quaternionf qset(1, 0, 0, 0);

    Matrix<float, Nx*Ny*3, 3> A;
    Matrix<float, Nx*Ny*3, 1> b;

    // begin rotation & data collection
    for (int i = 0; i < Nx; ++i)
    {
        cout << "[X rotation]: " << ang_x_array[i] << endl;
        aaX.angle() = ang_x_array[i];
        for (int j = 0; j < Ny; ++j)
        {
            cout << "   [Y rotation]: " << ang_y_array[j] << endl;
            aaY.angle() = ang_y_array[j];
            Quaternionf qr(aaX*aaY);

            qset    = quatMTimes(qr, q0);
            pose[3] = qset.w();
            pose[4] = qset.x();
            pose[5] = qset.y();
            pose[6] = qset.z();

            robot.setControl(pose);
            ros::Duration(2).sleep();

            robot.getState(pose, wrench);

            A.block<3,3>(3*i*Ny + 3*j, 0) <<          0,  wrench[2], -wrench[1],
                                             -wrench[2],          0,  wrench[0],
                                              wrench[1], -wrench[0],          0;
            b.block<3,1>(3*i*Ny + 3*j, 0) << wrench[3], wrench[4], wrench[5];
        }
    }

    cout << "[Data collection is done.]" << endl << endl;
    cout << "A: " << endl << A << endl;
    cout << "b: " << endl << b << endl;
    cout << "Solving SVD..." << endl;

    Vector3f x = A.JacobiSVD().solve(b);
    cout << "Done." << endl;
    cout << "X: " << x << endl;

    
    delete [] ang_x_array;
    delete [] ang_y_array;

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
    return 0;
}
