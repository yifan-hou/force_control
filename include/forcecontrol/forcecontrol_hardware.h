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
#include <cmath>
#include <unistd.h>
// #include <pthread.h>
// #include <memory>
// #include <boost/program_options.hpp>

#include <forcecontrol/ati_netft_hardware.h>
#include <egm_hardware/EGMClass.h>

// namespace po = boost::program_options;
using namespace std;


class ForceControlHardware : public hardware_interface::RobotHW
{
  public:
    ForceControlHardware();
    ~ForceControlHardware();
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    void getState(float *pose, float *wrench);
    void setControl(const float *pose_set);

    ATINetftHardware *ati;
    EGMClass *egm;

  private:

};
