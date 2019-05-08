/*
    roscontrol hardware_interface wrapper for force-control.
    Use EGM module and ATI Netft sensor module.

*/
#pragma once
#include <hardware_interface/robot_hw.h>
// #include <hardware_interface/force_torque_sensor_interface.h>

// #include <ros/ros.h>

#include <iostream>
#include <string>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <Eigen/Dense>

#include <forcecontrol/ati_netft_hardware.h>
#include <egm/EGMClass.h>



class ForceControlHardware : public hardware_interface::RobotHW
{
  public:
    ForceControlHardware();
    ~ForceControlHardware();
    bool init(ros::NodeHandle& root_nh, std::chrono::high_resolution_clock::time_point time0);
    void getPose(double *pose);
    bool getWrench(double *wrench); // measured in tool frame
    bool getState(double *pose, double *wrench);
    void setControl(const double *pose_set);

    ATINetftHardware *ati;
    EGMClass *egm;

    // parameters
    double *_WRENCH_SAFETY;

  private:
    Eigen::Vector3d _Foffset;
    Eigen::Vector3d _Toffset;
    Eigen::Vector3d _Gravity;
    Eigen::Vector3d _Pcom;
    Eigen::Matrix<double, 6, 6> _adj_sensor_tool;

};
