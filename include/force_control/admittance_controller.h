/*
This file is part of the package
https://github.com/yifan-hou/force_control

Reference: Y. Hou and M. T. Mason, "Robust Execution of Contact-Rich Motion Plans by Hybrid Force-Velocity Control,"
           2019 International Conference on Robotics and Automation (ICRA), Montreal, QC, Canada, 2019, pp. 1933-1939

The package is distributed under the following license:

MIT License

Copyright (c) [2024] [Yifan Hou]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#ifndef _ADMITTANCE_CONTROLLER_H_
#define _ADMITTANCE_CONTROLLER_H_

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <fstream>

class AdmittanceController {
 public:
  struct AdmittanceControllerConfig {
    double dt{0.002};  // used for integration/differentiation
    bool log_to_file{false};
    std::string log_file_path{""};
    bool alert_overrun{
        false};  // if true, print warning when step() takes too long

    struct ComplianceParameters6d {
      // Admittance parameters
      RUT::Matrix6d stiffness{};
      RUT::Matrix6d damping{};
      RUT::Matrix6d inertia{};
      RUT::Vector6d stiction{};  // static friction, eliminates drifting
    };
    ComplianceParameters6d compliance6d{};

    // only useful in tracking mode. This is damping on feedback velocity in addition to damping on error velocity
    RUT::Matrix6d additional_damping{RUT::Matrix6d::Zero()};

    // spring force will be capped at this value.
    double max_spring_force_magnitude{0.0};
    double max_spring_torque_magnitude{0.0};

    struct PIDGains {
      double P_trans{0.0};
      double I_trans{0.0};
      double D_trans{0.0};

      double P_rot{0.0};
      double I_rot{0.0};
      double D_rot{0.0};
    };
    PIDGains direct_force_control_gains{};
    RUT::Vector6d direct_force_control_I_limit{};
  };

  AdmittanceController();
  ~AdmittanceController();
  AdmittanceController(AdmittanceController&&);

  /**
   * @brief      initialize the controller.
   *
   * @param[in]  time0         The time point to start ticking from.
   * @param[in]  config        The struct contains all configs.
   * @param[in]  pose_current  The current robot pose (tool frame)
   *
   * @return     True if successfully initialized.
   */
  bool init(const RUT::TimePoint& time0,
            const AdmittanceControllerConfig& config,
            const RUT::Vector7d& pose_current);

  /**
   * @brief      Sets the robot status.
   *
   * @param[in]  pose_WT    The current tool frame pose in the world frame.
   * @param[in]  wrench_WT  The tool wrench feedback.
   */
  void setRobotStatus(const RUT::Vector7d& pose_WT,
                      const RUT::Vector6d& wrench_T);
  /**
   * @brief      Set the position and force reference (user command).
   *
   * @param[in] pose_WT     tool pose represented in the world frame.
   * @param[in] wrench_WTr  wrench measured in the transformed frame.
   *
   * WARNING remember to call step() after setRobotReference, before
   * setForceControlledAxis. step() will properly update internal states, which
   * is required for setForceControlledAxis to work properly.
   */
  void setRobotReference(const RUT::Vector7d& pose_WT,
                         const RUT::Vector6d& wrench_WTr);

  /**
   * @brief      Set the tracking reference for the robot.
   *
   * @param[in]  pose_WT     The desired tool pose in the world frame.
   * @param[in]  v_spatial_WT The desired spatial velocity in the world frame.
   * @param[in]  a_spatial_WT The desired time derivative of v_spatial_WT.
   * @param[in]  wrench_WTr  The desired wrench in the transformed frame.
   */
  void setRobotTrackingReference(const RUT::Vector7d& pose_WT,
                                 const RUT::Vector6d& v_spatial_WT,
                                 const RUT::Vector6d& a_spatial_WT,
                                 const RUT::Vector6d& wrench_WTr);
  /**
   * @brief      Sets the force controlled axis.
   *
   * @param[in]  Tr    6x6 orthonormal matrix. Describes the axis direction.
   * @param[in]  n_af  The number of force controlled axes.
   */
  void setForceControlledAxis(const RUT::Matrix6d& Tr, int n_af);

  /**
   * @brief      Sets the stiffness matrix.
   *
   * @param[in]  stiffness  The stiffness matrix.
   */
  void setStiffnessMatrix(const RUT::Matrix6d& stiffness);

  /**
   * @brief      Sets the damping matrix.
   *
   * @param[in]  damping  The damping matrix.
   */
  void setDampingMatrix(const RUT::Matrix6d& damping);

  /**
   * @brief return true if no error.
   */
  int step(RUT::Vector7d& pose);

  /**
   * @brief      Reset all internal states to default. It is recommended to call
   * reset() everytime the robot starts from a complete stop in the air. This
   * includes setting all position offsets/force errors to zero. Call reset()
   * when the next action is computed based on the robot's current pose instead
   *  of being part of a pre-planned trajectory. After a reset(), call
   * setRobotReference() immediately.
   */
  void reset();

  /**
   * @brief      Print the current states to the console.
   */
  void displayStates();

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif  // _ADMITTANCE_CONTROLLER_H_
