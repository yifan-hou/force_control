#pragma once
#ifndef _FORCE_CONTROL_CONTROLLER_H_
#define _FORCE_CONTROL_CONTROLLER_H_


#include <fstream>
#include <chrono>
#include <deque>

#include <Eigen/Geometry>

#include <force_control/force_control_hardware.h>

enum HYBRID_SERVO_MODE {
  HS_STOP_AND_GO,
  HS_CONTINUOUS
};

/**
 * Class for performing 6-axis Cartesian hybrid force-velocity control. The
 * force control is implemented as direct force control with position-control
 * inner loop and force feedback. The class interfaces with the hardwares via
 * class ForceControlHardware.
 *
 * Usage:
 *  Initialize:
 *    1. Call Init()
 *    2. Call updateAxis()
 *  Do control
 *    Call update() at your desired frequency.
 *    Call setPose(), setForce() and updateAxis() whenever needed.
 *    Note: after calling setPose(), you must call update() before updateAxis()
 *  Reset offset (and internal states)
 *    If you have a complete stop during execution and would like to restart
 *    from there, call reset() then updateAxis().
 *
 */
class ForceControlController
{
public:
  ForceControlController();
  ~ForceControlController();
  bool init(ros::NodeHandle& root_nh, ForceControlHardware *hw, std::chrono::high_resolution_clock::time_point time0);
  ///
  /// Reset the internal state variables in the control law, including setting
  ///     all position offsets/force errors to zero. "previous pose command" is
  ///     set to the robot's current pose. "Current velocity" is set to zero.
  /// It's as if the robot was commanded to its current pose and stabilized.
  /// Call reset() only if the next action is computed based on the robot's
  ///     current pose instead of being part of a pre-planned trajectory.
  void reset();
  void displayStates();

  /* Low level interfaces (no motion planning) */

  ///
  /// Set the pose command. Next update() will use this command.
  /// WARNING remember to call update() after setPose, before updateAxis.
  ///
  void setPose(const double *pose);
  void setForce(const double *force);
  void getPose(double *pose);
  void getToolVelocity(Eigen::Matrix<double, 6, 1> *v_T);
  bool getToolWrench(Eigen::Matrix<double, 6, 1> *wrench);

  bool update();
  void updateAxis(const Eigen::Matrix<double, 6, 6> &T, int n_af);

  /* Middle level interfaces (with motion planning) */

  /**
   * Execute a 6D HFVC command. The number of timesteps is determined by @p
   * main_loop_rate and @p duration.
   *
   * There are two modes: HS_STOP_AND_GO: user recomputes @p pose_set based on
   * robot's current pose feedback before calling this function. HS_CONTINUOUS:
   * user is sending a pre-computed trajectory frame by frame. The difference is
   * that, HS_STOP_AND_GO mode will reset every time you call this function. The
   * two modes are the same if there is no motion in the force controlled
   * directions. (e.g. n_af = 0)
   *
   * @param[in]  n_af            The dimension of force command
   * @param[in]  n_av            The dimension of velocity command
   * @param[in]  R_a             The orientation of hybrid force-velocity
   *                             control actions.
   * @param[in]  pose_set        7x1 goal pose (x y z qw qx qy qz), described in
   *                             the world frame
   * @param[in]  force_set       6x1 force vector, described in the transformed
   *                             action space
   * @param[in]  mode            HS_STOP_AND_GO or HS_CONTINUOUS.
   * @param[in]  main_loop_rate  The main loop rate in Hz
   * @param[in]  duration        The duration in seconds
   *
   * @return     true if success
   */
  bool ExecuteHFVC(const int n_af, const int n_av,
      const Eigen::Matrix<double, 6, 6> &R_a, const double *pose_set,
      const double *force_set,
      HYBRID_SERVO_MODE mode, const int main_loop_rate, const double duration);


  // parameters
  double _dt; // used for integration/differentiation
  Eigen::Matrix<double, 6, 6> _ToolStiffnessMatrix;
  Eigen::Matrix<double, 6, 6> _ToolDamping_coef;
  Eigen::Matrix<double, 6, 6> _ToolInertiaMatrix;
  double _kForceControlPGainTran, _kForceControlIGainTran, _kForceControlDGainTran;
  double _kForceControlPGainRot, _kForceControlIGainRot, _kForceControlDGainRot;
  Eigen::Matrix<double, 6, 1> _FC_I_limit_T_6D;
  // speed limits. mm/s, mm/s^2, rad/s, rad/s^2
  double _kAccMaxTrans, _kVelMaxTrans, _kAccMaxRot, _kVelMaxRot;

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
  Eigen::Matrix<double, 6, 1> _v_T;
  Eigen::Matrix<double, 6, 1> _wrench_T_Err;
  Eigen::Matrix<double, 6, 1> _wrench_T_Err_I;

  // experimental
  bool _activate_experimental_feature;
  int _pool_size;
  std::deque<Eigen::Matrix<double, 6, 1>> _f_queue;
  std::deque<double> _f_weights;
  std::deque<double> _f_probability;
  std::deque<Eigen::Matrix<double, 6, 1>> _v_queue;
  std::deque<double> _v_weights;
  std::deque<double> _v_probability;
  std::vector<double> _scale_force_vector;
  std::vector<double> _scale_vel_vector;
  double _var_force, _var_velocity;


private:
  ForceControlHardware *_hw;

  // misc
  Clock::time_point _time0; ///< high resolution timer.
  std::ofstream _file;
  bool _print_flag;

};

#endif // _FORCE_CONTROL_CONTROLLER_H_
