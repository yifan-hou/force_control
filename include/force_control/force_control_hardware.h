/*
    Hardware wrapper for force-control.
    Includes a robot arm and a wrist-mounted ft sensor.
*/
#pragma once
// #include <hardware_interface/robot_hw.h>

#include <iostream>
#include <string>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <hardware_interfaces/robot_interfaces.h>
#include <hardware_interfaces/ft_interfaces.h>

typedef std::chrono::high_resolution_clock Clock;

class ForceControlHardware {
  public:
    ForceControlHardware();
    ~ForceControlHardware();
    bool init(ros::NodeHandle& root_nh, Clock::time_point time0,
        FTInterfaces *ft, RobotInterfaces *robot);
    void getPose(double *pose);
    bool getWrench(double *wrench); // get the wrench in tool frame
    bool getState(double *pose, double *wrench);
    void setPose(const double *pose_set);

    FTInterfaces *_ft;
    RobotInterfaces *_robot;
};
