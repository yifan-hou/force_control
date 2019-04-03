/// Class for performing 6-axis Cartesian hybrid force-velocity control
/// with Cartesian position-control inner loop and wrist-mounted FT sensor.
///
/// Usage:
///   Initialize:
///     1. Call Init()
///     2. Call updateAxis()
///   Do control
///     Call update() at your desired frequency.
///     Call setPose(), setForce() and updateAxis() whenever needed.
///     Note: after calling setPose(), you must call update() before updateAxis()
///   Reset offset (and internal states)
///     If you have a complete stop during execution and would like to restart
///     from there, call reset() then updateAxis().
#pragma once
#ifndef _FORCECONTROL_CONTROLLER_H_
#define _FORCECONTROL_CONTROLLER_H_


#include <fstream>
#include <chrono>

#include <Eigen/Geometry>

#include <forcecontrol/forcecontrol_hardware.h>


class ForceControlController
{
public:
  ForceControlController();
  ~ForceControlController();
  bool init(ros::NodeHandle& root_nh, ForceControlHardware *hw, std::chrono::high_resolution_clock::time_point time0);

  ///
  /// Set the pose command. Next update() will use this command.
  /// WARNING remember to call update() after setPose, before updateAxis.
  ///
  void setPose(const double *pose);
  void setForce(const double *force);
  bool update(const ros::Time& time, const ros::Duration& period);
  void updateAxis(const Eigen::Matrix<double, 6, 6> &T, int n_af);
  ///
  /// Reset the internal state variables in the control law, including setting
  ///     all position offsets/force errors to zero. "previous pose command" is
  ///     set to the robot's current pose. "Current velocity" is set to zero.
  /// It's as if the robot was commanded to its current pose and stabilized.
  /// Call reset() only if the next action is computed based on the robot's
  ///     current pose instead of being part of a pre-planned trajectory.
  void reset();
  void displayStates();

  // parameters
  double _dt; // used for integration/differentiation
  Eigen::Matrix<double, 6, 6> _ToolStiffnessMatrix;
  Eigen::Matrix<double, 6, 6> _ToolDamping_coef;
  Eigen::Matrix<double, 6, 6> _ToolInertiaMatrix;
  double _kForceControlPGainTran, _kForceControlIGainTran, _kForceControlDGainTran;
  double _kForceControlPGainRot, _kForceControlIGainRot, _kForceControlDGainRot;
  Eigen::Matrix<double, 6, 1> _FC_I_limit_T_6D;

  // commands
  double *_pose_user_input;
  Eigen::Matrix<double, 6, 1> _wrench_Tr_set;
  Eigen::Matrix<double, 6, 6> _Tr;
  Eigen::Matrix<double, 6, 6> _Tr_inv;
  Eigen::Matrix<double, 6, 6> _m_force_selection;
  Eigen::Matrix<double, 6, 6> _m_velocity_selection;


  // Controller internal
  double *_pose_sent_to_robot;
  Eigen::Matrix4d _SE3_WT_old;
  Eigen::Matrix4d _SE3_WToffset;
  Eigen::Matrix<double, 6, 1> _v_W;
  Eigen::Matrix<double, 6, 1> _wrench_T_Err;
  Eigen::Matrix<double, 6, 1> _wrench_T_Err_I;


private:
  ForceControlHardware *_hw;

  // misc
  std::chrono::high_resolution_clock::time_point _time0; ///< high resolution timer.
  ofstream _file;
  bool _print_flag;

};

#endif // _FORCECONTROL_CONTROLLER_H_
