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

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <force_control/admittance_controller.h>

#include <Eigen/QR>
#include <cmath>
#include <iostream>
#include <string>

using RUT::Matrix4d;
using RUT::Matrix6d;
using RUT::MatrixXd;
using RUT::Vector6d;

Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
                          "]");

struct AdmittanceController::Implementation {
  Implementation();
  ~Implementation();
  bool initialize(
      const RUT::TimePoint& time_initial_0,
      const AdmittanceControllerConfig& admittance_controller_config);

  void setRobotStatus(const RUT::Vector7d& pose_WT,
                      const RUT::Vector6d& wrench_WT);
  void setRobotReference(const RUT::Vector7d& pose_WT,
                         const RUT::Vector6d& wrench_WTr);
  void setRobotTrackingReference(const RUT::Vector7d& pose_WT,
                                 const RUT::Vector6d& v_spatial_WT,
                                 const RUT::Vector6d& a_spatial_WT,
                                 const RUT::Vector6d& wrench_WT);
  void setForceControlledAxis(const Matrix6d& Tr_new, int n_af);
  void setStiffnessMatrix(const Matrix6d& stiffness);
  void setDampingMatrix(const Matrix6d& damping);
  int step(RUT::Vector7d& pose_to_send);
  void reset();
  void logStates();
  void displayStates() const;

  AdmittanceControllerConfig config{};

  // internal controller states
  Matrix6d Tr{};
  Matrix6d Tr_inv{};
  Vector6d v_force_selection{};
  Vector6d v_velocity_selection{};
  Matrix6d diag_force_selection{};
  Matrix6d diag_velocity_selection{};
  Matrix6d m_anni{};

  Matrix4d SE3_WTref{};
  Matrix4d SE3_WT{};
  Matrix4d SE3_TrefTadj{};
  Matrix4d SE3_WTadj{};
  Matrix4d SE3_TTadj{};
  Matrix4d SE3_WT_cmd{};
  Vector6d spt_TTadj{};
  Vector6d spt_TTadj_new{};
  Matrix6d Adj_WT{};
  Matrix6d Adj_TW{};
  Matrix6d Adj_TrefW{};
  Matrix6d Adj_TTref{};
  Matrix6d Jac_v_spt{};
  Matrix6d Jac_v_spt_inv{};

  Vector6d v_body_WTref{};
  Vector6d a_body_WTref{};
  Vector6d v_spatial_WT{};
  Vector6d v_body_WT{};
  Vector6d v_body_WT_vel_ref{};
  Vector6d v_body_TrefT{};
  Vector6d v_Tr{};
  Vector6d vd_Tr{};
  Vector6d wrench_T_Err_prev{};
  Vector6d wrench_T_Err_I{};
  Vector6d err_vd_Tr{};
  Vector6d a_Tr_ref{};

  Vector6d wrench_T_fb{};  // force feedback measured in tool frame
  Vector6d wrench_Tr_cmd{};
  Vector6d wrench_T_spring{};
  Vector6d wrench_Tr_spring{};
  Vector6d wrench_Tr_fb{};
  Vector6d wrench_T_cmd{};
  Vector6d wrench_T_Err{};
  Vector6d wrench_T_PID{};
  Vector6d wrench_Tr_PID{};
  Vector6d wrench_Tr_Err{};
  Vector6d wrench_Tr_damping{};
  Vector6d wrench_Tr_All{};

  // misc
  RUT::Timer timer{};
  RUT::Profiler profiler{};
  std::ofstream log_file{};
};

AdmittanceController::Implementation::Implementation() {}

AdmittanceController::Implementation::~Implementation() {
  if (config.log_to_file)
    log_file.close();
}

bool AdmittanceController::Implementation::initialize(
    const RUT::TimePoint& time_initial_0,
    const AdmittanceControllerConfig& admittance_controller_config) {
  std::cout << "[AdmittanceController] Begin initialization.\n";
  config = admittance_controller_config;
  timer.tic(time_initial_0);

  reset();

  if (config.log_to_file) {
    log_file.open(config.log_file_path);
    if (log_file.is_open())
      std::cout << "[AdmittanceController] log file opened successfully at "
                << config.log_file_path << std::endl;
    else
      std::cerr << "[AdmittanceController] Failed to open log file at "
                << config.log_file_path << std::endl;
  }
  return true;
}

void AdmittanceController::Implementation::setRobotStatus(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WT) {
  SE3_WT = RUT::pose2SE3(pose_WT);
  wrench_T_fb = -wrench_WT;
}

void AdmittanceController::Implementation::setRobotReference(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WTr) {
  SE3_WTref = RUT::pose2SE3(pose_WT);
  wrench_Tr_cmd = wrench_WTr;
}

void AdmittanceController::Implementation::setRobotTrackingReference(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& v_body_WT,
    const RUT::Vector6d& a_body_WT, const RUT::Vector6d& wrench_WTr) {
  SE3_WTref = RUT::pose2SE3(pose_WT);
  v_body_WTref = v_body_WT;
  a_body_WTref = a_body_WT;
  wrench_Tr_cmd = wrench_WTr;
}

// After axis update, the goal pose with offset should be equal to current pose
// in the new velocity controlled axes. To satisfy this requirement, we need to
// change SE3_TrefTadj accordingly
void AdmittanceController::Implementation::setForceControlledAxis(
    const Matrix6d& Tr_new, int n_af) {
  v_force_selection = Vector6d::Zero();
  v_velocity_selection = Vector6d::Ones();
  for (int i = 0; i < n_af; ++i) {
    v_force_selection(i) = 1;
    v_velocity_selection(i) = 0;
  }
  diag_force_selection = v_force_selection.asDiagonal();
  diag_velocity_selection = v_velocity_selection.asDiagonal();

  m_anni = diag_velocity_selection * Tr * Jac_v_spt;
  spt_TTadj_new =
      (Matrix6d::Identity() - RUT::pseudoInverse(m_anni, 1e-6) * m_anni) *
      spt_TTadj;
  SE3_TrefTadj = SE3_WT * RUT::spt2SE3(spt_TTadj_new) * RUT::SE3Inv(SE3_WTref);

  // project these into force space
  wrench_T_Err_I = Tr_inv * diag_force_selection * Tr * wrench_T_Err_I;
  wrench_T_Err_prev = Tr_inv * diag_force_selection * Tr * wrench_T_Err_prev;

  Tr = Tr_new;
  Tr_inv = Tr.inverse();

  if (std::isnan(SE3_TrefTadj(0, 0))) {
    std::cerr << "\nThe computed offset has NaN." << std::endl;
    std::cerr << "SE3_WT:\n" << SE3_WT.format(MatlabFmt) << std::endl;
    std::cerr << "SE3_TTadj:\n" << SE3_TTadj.format(MatlabFmt) << std::endl;
    std::cerr << "spt_TTadj:\n" << spt_TTadj.format(MatlabFmt) << std::endl;
    std::cerr << "Jac_v_spt_inv:\n"
              << Jac_v_spt_inv.format(MatlabFmt) << std::endl;
    std::cerr << "Jac_v_spt:\n" << Jac_v_spt.format(MatlabFmt) << std::endl;
    std::cerr << "m_anni:\n" << m_anni.format(MatlabFmt) << std::endl;
    std::cerr << "spt_TTadj_new:\n"
              << spt_TTadj_new.format(MatlabFmt) << std::endl;
    std::cerr << "SE3_TrefTadj:\n"
              << SE3_TrefTadj.format(MatlabFmt) << std::endl;
    std::cerr << "\nNow paused at setForceControlledAxis()";
    getchar();
  }
}

void AdmittanceController::Implementation::setStiffnessMatrix(
    const Matrix6d& stiffness) {
  config.compliance6d.stiffness = stiffness;
}

void AdmittanceController::Implementation::setDampingMatrix(
    const Matrix6d& damping) {
  config.compliance6d.damping = damping;
}

// clang-format off
/*
 *
    force control law
        Frames:
            W: world frame
            T: current tool frame
            Tr: transformed generalized space
        Frame suffixes
            fb: feedback (default, often omitted)
            ref: user provided reference, target
            adj: offset adjusted (command tool frame)
            cmd: output command, to be sent to the robot
        Quantities:
            SE3: 4x4 homogeneous coordinates
            se3: 6x1 twist coordinate of SE3
            spt: 6x1 special twist: 3x1 position, 3x1 exponential coordinate for rotation
            v: 6x1 velocity measured in an inertia frame.
              v_body: body velocity.
              v_spatial: spatial velocity.
            wrench: 6x1 wrench. Makes work with body velocity
            Jac_v_spt: 6x6 jacobian from body velocity to spt:
                Jac_v_spt * body velocity = spt time derivative
            Tr: 6x6 transformation matrix. Describes the force-velocity decomposition
 *
 */
// clang-format on
int AdmittanceController::Implementation::step(RUT::Vector7d& pose_to_send) {
  profiler.clear();
  profiler.start();
  timer.tic();
  // ----------------------------------------
  //  Compute Forces in Generalized space
  // ----------------------------------------
  /* Position updates */
  SE3_WTadj = SE3_WTref * SE3_TrefTadj;
  SE3_TTadj = RUT::SE3Inv(SE3_WT) * SE3_WTadj;  // aka SE3_S_err
  spt_TTadj = RUT::SE32spt(SE3_TTadj);

  Jac_v_spt_inv = RUT::JacobianSpt2BodyV(SE3_WT.block<3, 3>(0, 0));
  Jac_v_spt = Jac_v_spt_inv.inverse();

  Adj_WT = RUT::SE32Adj(SE3_WT);
  Adj_TW = RUT::SE32Adj(RUT::SE3Inv(SE3_WT));
  Adj_TrefW = RUT::SE32Adj(RUT::SE3Inv(SE3_WTref));
  Adj_TTref = RUT::SE32Adj(RUT::SE3Inv(SE3_WT) * SE3_WTref);

  /* Velocity updates */
  v_body_WT = Adj_TW * v_spatial_WT;

  v_body_TrefT = v_body_WT - Adj_TTref * v_body_WTref;  // e_dot
  v_Tr = Tr * v_body_TrefT;

  /* Wrench updates */
  wrench_T_spring = Jac_v_spt * config.compliance6d.stiffness * spt_TTadj;
  profiler.stop("1");
  profiler.start();

  // clip spring force
  if (config.max_spring_force_magnitude > 0) {
    double spring_force_magnitude = wrench_T_spring.head<3>().norm();
    if (spring_force_magnitude > config.max_spring_force_magnitude) {
      wrench_T_spring.head<3>() *=
          config.max_spring_force_magnitude / spring_force_magnitude;
    }
  }
  if (config.max_spring_torque_magnitude > 0) {
    double spring_torque_magnitude = wrench_T_spring.tail<3>().norm();
    if (spring_torque_magnitude > config.max_spring_torque_magnitude) {
      wrench_T_spring.tail<3>() *=
          config.max_spring_torque_magnitude / spring_torque_magnitude;
    }
  }

  wrench_Tr_spring = Tr * wrench_T_spring;

  /* Force error, PID force control */
  wrench_T_cmd = Tr_inv * wrench_Tr_cmd;
  wrench_T_Err = wrench_T_cmd - wrench_T_fb;
  wrench_T_Err_I += wrench_T_Err;
  RUT::truncate6d(&wrench_T_Err_I, -config.direct_force_control_I_limit,
                  config.direct_force_control_I_limit);

  wrench_T_PID.head(3) =
      config.direct_force_control_gains.P_trans * wrench_T_Err.head(3) +
      config.direct_force_control_gains.I_trans * wrench_T_Err_I.head(3) +
      config.direct_force_control_gains.D_trans *
          (wrench_T_Err.head(3) - wrench_T_Err_prev.head(3));
  wrench_T_PID.tail(3) =
      config.direct_force_control_gains.P_rot * wrench_T_Err.tail(3) +
      config.direct_force_control_gains.I_rot * wrench_T_Err_I.tail(3) +
      config.direct_force_control_gains.D_rot *
          (wrench_T_Err.tail(3) - wrench_T_Err_prev.tail(3));
  wrench_Tr_PID = Tr * wrench_T_PID;
  wrench_T_Err_prev = wrench_T_Err;
  wrench_Tr_Err = Tr * wrench_T_Err;

  /* Apply static friction */
  for (int i = 0; i < 6; ++i) {
    if (std::abs(wrench_Tr_Err(i)) < config.compliance6d.stiction(i)) {
      wrench_Tr_Err(i) = 0;
    }
  }

  wrench_Tr_damping = -Tr * (config.compliance6d.damping * v_body_TrefT +
                             config.additional_damping * v_body_WT);

  wrench_Tr_All = diag_force_selection * (wrench_Tr_spring + wrench_Tr_Err +
                                          wrench_Tr_PID + wrench_Tr_damping);
  profiler.stop("2");
  profiler.start();
  // ----------------------------------------
  //  force to velocity
  // ----------------------------------------

  /* Newton's Law */
  //  Axes are no longer independent when we take
  //      rotation in to consideration.
  //  Newton's Law in body (Tool) frame:
  //      W=M*vd
  //          W: body wrench
  //          M: Inertia matrix in body frame
  //          vd: body velocity time derivative
  //  Newton's law in transformed space
  //      TW=TMTinv Tvd
  //      W_Tr = TMTinv vd_Tr
  err_vd_Tr = (Tr * config.compliance6d.inertia * Tr_inv)
                  .fullPivLu()
                  .solve(wrench_Tr_All);
  a_Tr_ref = Tr * a_body_WTref;
  vd_Tr = err_vd_Tr + a_Tr_ref;

  // Velocity in the force-controlled direction: integrate acc computed from
  // Newton's law
  v_Tr += config.dt * vd_Tr;  // for tracking, v_Tr is error_d_Tr
  v_Tr = diag_force_selection *
         v_Tr;  // clean up velocity in the velocity-controlled direction

  // Velocity in the velocity-controlled direction: derive from reference pose
  // No mass/spring/damping, just go there directly
  v_body_WT_vel_ref =
      Jac_v_spt_inv * spt_TTadj /
      config.dt;  // reference velocity, derived from reference pose
  v_Tr += diag_velocity_selection * Tr * v_body_WT_vel_ref;

  v_spatial_WT = Adj_WT * Tr_inv * v_Tr;
  profiler.stop("3");
  profiler.start();
  // ----------------------------------------
  //  velocity to pose
  // ----------------------------------------
  SE3_WT_cmd = SE3_WT + RUT::wedge6(v_spatial_WT) * SE3_WT * config.dt;
  RUT::SE32Pose(SE3_WT_cmd, pose_to_send);

  if (std::isnan(pose_to_send[0])) {
    std::cerr << "==================== pose is nan. =====================\n";
    displayStates();
    std::cerr << "Press ENTER to continue..." << std::endl;
    getchar();
    return false;
  }
  profiler.stop("4");
  profiler.start();
  if (config.log_to_file) {
    logStates();
  }
  profiler.stop("5");
  double timenow = timer.toc_ms();
  if (config.alert_overrun && timenow > config.dt * 1000.) {
    std::cerr << "AdmittanceController: step took too long: " << timenow << "ms"
              << std::endl;
    std::cerr << "Profiler: " << std::endl;
    profiler.show();
  }

  return true;
}

void AdmittanceController::Implementation::reset() {
  Tr = Matrix6d::Identity();
  Tr_inv = Matrix6d::Identity();
  v_force_selection = Vector6d::Zero();
  v_velocity_selection = Vector6d::Ones();
  diag_force_selection = Matrix6d::Zero();
  diag_velocity_selection = Matrix6d::Identity();
  m_anni = Matrix6d::Identity();

  SE3_WTref = Matrix4d::Identity();
  SE3_WT = Matrix4d::Identity();
  SE3_TrefTadj = Matrix4d::Identity();
  SE3_WTadj = Matrix4d::Identity();
  SE3_TTadj = Matrix4d::Identity();
  SE3_WT_cmd = Matrix4d::Identity();
  spt_TTadj = Vector6d::Zero();
  spt_TTadj_new = Vector6d::Zero();
  Adj_WT = Matrix6d::Identity();
  Adj_TW = Matrix6d::Identity();
  Adj_TrefW = Matrix6d::Identity();
  Adj_TTref = Matrix6d::Identity();
  Jac_v_spt = Matrix6d::Identity();
  Jac_v_spt_inv = Matrix6d::Identity();

  v_body_WTref = Vector6d::Zero();
  a_body_WTref = Vector6d::Zero();
  v_spatial_WT = Vector6d::Zero();
  v_body_WT = Vector6d::Zero();
  v_body_WT_vel_ref = Vector6d::Zero();
  v_body_TrefT = Vector6d::Zero();
  v_Tr = Vector6d::Zero();
  vd_Tr = Vector6d::Zero();
  wrench_T_Err_prev = Vector6d::Zero();
  wrench_T_Err_I = Vector6d::Zero();
  err_vd_Tr = Vector6d::Zero();
  a_Tr_ref = Vector6d::Zero();

  wrench_T_fb = Vector6d::Zero();
  wrench_Tr_cmd = Vector6d::Zero();
  wrench_T_spring = Vector6d::Zero();
  wrench_Tr_spring = Vector6d::Zero();
  wrench_Tr_fb = Vector6d::Zero();
  wrench_T_cmd = Vector6d::Zero();
  wrench_T_Err = Vector6d::Zero();
  wrench_T_PID = Vector6d::Zero();
  wrench_Tr_PID = Vector6d::Zero();
  wrench_Tr_Err = Vector6d::Zero();
  wrench_Tr_damping = Vector6d::Zero();
  wrench_Tr_All = Vector6d::Zero();
}

void AdmittanceController::Implementation::logStates() {
  log_file << timer.toc_ms() << " ";
  RUT::stream_array_in(log_file, SE3_WTref.block<3, 1>(0, 3), 3);
  RUT::stream_array_in(log_file, SE3_WT.block<3, 1>(0, 3), 3);
  RUT::stream_array_in(log_file, SE3_WTadj.block<3, 1>(0, 3), 3);
  RUT::stream_array_in(log_file, SE3_WT_cmd.block<3, 1>(0, 3), 3);
  RUT::stream_array_in6d(log_file, wrench_T_fb);
  RUT::stream_array_in6d(log_file, wrench_Tr_All);

  // SE3_TrefTadj = Matrix4d::Identity();
  // SE3_TTadj = Matrix4d::Identity();
  // spt_TTadj = Vector6d::Zero();
  // spt_TTadj_new = Vector6d::Zero();
  // Adj_WT = Matrix6d::Identity();
  // Adj_TW = Matrix6d::Identity();
  // Jac_v_spt = Matrix6d::Identity();
  // Jac_v_spt_inv = Matrix6d::Identity();

  // v_spatial_WT = Vector6d::Zero();
  // v_body_WT = Vector6d::Zero();
  // v_body_WT_vel_ref = Vector6d::Zero();
  // v_Tr = Vector6d::Zero();
  // vd_Tr = Vector6d::Zero();
  // wrench_T_Err_prev = Vector6d::Zero();
  // wrench_T_Err_I = Vector6d::Zero();

  // wrench_T_fb = Vector6d::Zero();
  // wrench_Tr_cmd = Vector6d::Zero();
  // wrench_T_spring = Vector6d::Zero();
  // wrench_Tr_spring = Vector6d::Zero();
  // wrench_Tr_fb = Vector6d::Zero();
  // wrench_T_cmd = Vector6d::Zero();
  // wrench_T_Err = Vector6d::Zero();
  // wrench_T_PID = Vector6d::Zero();
  // wrench_Tr_PID = Vector6d::Zero();
  // wrench_Tr_Err = Vector6d::Zero();
  // wrench_Tr_damping = Vector6d::Zero();
  // wrench_Tr_All = Vector6d::Zero();

  log_file << std::endl;
}

void AdmittanceController::Implementation::displayStates() const {
  std::cout << "================= Parameters ================== " << std::endl;
  std::cout << "dt: " << config.dt << std::endl;
  std::cout << "log_to_file: " << config.log_to_file << std::endl;
  std::cout << "log_file_path: " << config.log_file_path << std::endl;
  std::cout << "compliance6d.stiffness: "
            << config.compliance6d.stiffness.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.damping: "
            << config.compliance6d.damping.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.inertia: "
            << config.compliance6d.inertia.format(MatlabFmt) << std::endl;
  std::cout << "direct_force_control_gains.P_trans: "
            << config.direct_force_control_gains.P_trans << std::endl;
  std::cout << "direct_force_control_gains.I_trans: "
            << config.direct_force_control_gains.I_trans << std::endl;
  std::cout << "direct_force_control_gains.D_trans: "
            << config.direct_force_control_gains.D_trans << std::endl;
  std::cout << "direct_force_control_gains.P_rot: "
            << config.direct_force_control_gains.P_rot << std::endl;
  std::cout << "direct_force_control_gains.I_rot: "
            << config.direct_force_control_gains.I_rot << std::endl;
  std::cout << "direct_force_control_gains.D_rot: "
            << config.direct_force_control_gains.D_rot << std::endl;
  std::cout << "direct_force_control_I_limit: "
            << config.direct_force_control_I_limit.format(MatlabFmt)
            << std::endl;
  std::cout << "================= Internal states ================== "
            << std::endl;
  std::cout << "Tr: " << Tr.format(MatlabFmt) << std::endl;
  std::cout << "Tr_inv: " << Tr_inv.format(MatlabFmt) << std::endl;
  std::cout << "v_force_selection: " << v_force_selection.format(MatlabFmt)
            << std::endl;
  std::cout << "v_velocity_selection: "
            << v_velocity_selection.format(MatlabFmt) << std::endl;
  std::cout << "diag_force_selection: "
            << diag_force_selection.format(MatlabFmt) << std::endl;
  std::cout << "diag_velocity_selection: "
            << diag_velocity_selection.format(MatlabFmt) << std::endl;
  std::cout << "m_anni: " << m_anni.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WTref: " << SE3_WTref.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WT: " << SE3_WT.format(MatlabFmt) << std::endl;
  std::cout << "SE3_TrefTadj: " << SE3_TrefTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WTadj: " << SE3_WTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_TTadj: " << SE3_TTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WT_cmd: " << SE3_WT_cmd.format(MatlabFmt) << std::endl;
  std::cout << "spt_TTadj: " << spt_TTadj.format(MatlabFmt) << std::endl;
  std::cout << "spt_TTadj_new: " << spt_TTadj_new.format(MatlabFmt)
            << std::endl;
  std::cout << "Adj_WT: " << Adj_WT.format(MatlabFmt) << std::endl;
  std::cout << "Adj_TW: " << Adj_TW.format(MatlabFmt) << std::endl;
  std::cout << "Jac_v_spt: " << Jac_v_spt.format(MatlabFmt) << std::endl;
  std::cout << "Jac_v_spt_inv: " << Jac_v_spt_inv.format(MatlabFmt)
            << std::endl;
  std::cout << "v_body_WTref: " << v_body_WTref.format(MatlabFmt) << std::endl;
  std::cout << "a_body_WTref: " << a_body_WTref.format(MatlabFmt) << std::endl;
  std::cout << "v_spatial_WT: " << v_spatial_WT.format(MatlabFmt) << std::endl;
  std::cout << "v_body_WT: " << v_body_WT.format(MatlabFmt) << std::endl;
  std::cout << "v_body_WT_vel_ref: " << v_body_WT_vel_ref.format(MatlabFmt)
            << std::endl;
  std::cout << "v_body_TrefT: " << v_body_TrefT.format(MatlabFmt) << std::endl;
  std::cout << "v_Tr: " << v_Tr.format(MatlabFmt) << std::endl;
  std::cout << "vd_Tr: " << vd_Tr.format(MatlabFmt) << std::endl;
  std::cout << "err_vd_Tr: " << err_vd_Tr.format(MatlabFmt) << std::endl;
  std::cout << "a_Tr_ref: " << a_Tr_ref.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_Err_prev: " << wrench_T_Err_prev.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_Err_I: " << wrench_T_Err_I.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_fb: " << wrench_T_fb.format(MatlabFmt) << std::endl;
  std::cout << "wrench_Tr_cmd: " << wrench_Tr_cmd.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_spring: " << wrench_T_spring.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_spring: " << wrench_Tr_spring.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_fb: " << wrench_Tr_fb.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_cmd: " << wrench_T_cmd.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_Err: " << wrench_T_Err.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_PID: " << wrench_T_PID.format(MatlabFmt) << std::endl;
  std::cout << "wrench_Tr_PID: " << wrench_Tr_PID.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_Err: " << wrench_Tr_Err.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_damping: " << wrench_Tr_damping.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_All: " << wrench_Tr_All.format(MatlabFmt)
            << std::endl;
}

AdmittanceController::AdmittanceController()
    : m_impl{std::make_unique<Implementation>()} {}
AdmittanceController::~AdmittanceController() = default;
AdmittanceController::AdmittanceController(AdmittanceController&&) = default;

bool AdmittanceController::init(const RUT::TimePoint& time0,
                                const AdmittanceControllerConfig& config,
                                const RUT::Vector7d& pose_current) {
  m_impl->initialize(time0, config);

  setRobotStatus(pose_current, RUT::Vector6d::Zero());
  setRobotReference(pose_current, RUT::Vector6d::Zero());

  RUT::Vector7d pose_out;
  step(pose_out);
  setForceControlledAxis(Matrix6d::Identity(), 0);

  std::cout << "[AdmittanceController] initialization is done." << std::endl;
  return true;
}

void AdmittanceController::setRobotStatus(const RUT::Vector7d& pose_WT,
                                          const RUT::Vector6d& wrench_WT) {
  m_impl->setRobotStatus(pose_WT, wrench_WT);
}

void AdmittanceController::setRobotReference(const RUT::Vector7d& pose_WT,
                                             const RUT::Vector6d& wrench_WTr) {
  m_impl->setRobotReference(pose_WT, wrench_WTr);
}

void AdmittanceController::setRobotTrackingReference(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& v_body_WT,
    const RUT::Vector6d& a_body_WT, const RUT::Vector6d& wrench_WT) {
  m_impl->setRobotTrackingReference(pose_WT, v_body_WT, a_body_WT,
                                    wrench_WT);
}

void AdmittanceController::setForceControlledAxis(const Matrix6d& Tr_new,
                                                  int n_af) {
  m_impl->setForceControlledAxis(Tr_new, n_af);
}

void AdmittanceController::setStiffnessMatrix(const Matrix6d& stiffness) {
  m_impl->setStiffnessMatrix(stiffness);
}

void AdmittanceController::setDampingMatrix(const Matrix6d& damping) {
  m_impl->setDampingMatrix(damping);
}

int AdmittanceController::step(RUT::Vector7d& pose_to_send) {
  return m_impl->step(pose_to_send);
}

void AdmittanceController::displayStates() const {
  m_impl->displayStates();
}