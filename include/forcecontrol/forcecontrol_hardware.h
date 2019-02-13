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
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <forcecontrol/ati_netft_hardware.h>
#include <egm/EGMClass.h>



class ForceControlHardware : public hardware_interface::RobotHW
{
  public:
    ForceControlHardware();
    ~ForceControlHardware();
    bool init(ros::NodeHandle& root_nh, std::chrono::high_resolution_clock::time_point time0);
    void getPose(float *pose);
    bool getWrench(float *wrench);
    bool getState(float *pose, float *wrench);
    void setControl(const float *pose_set);

    void liftup(const float dz); // move up for safety
    ATINetftHardware *ati;
    EGMClass *egm;

    // parameters
    float *_WRENCH_SAFETY;

  private:
    Eigen::Vector3f _Foffset;
    Eigen::Vector3f _Toffset;
    Eigen::Vector3f _Gravity;
    Eigen::Vector3f _Pcom;

};
