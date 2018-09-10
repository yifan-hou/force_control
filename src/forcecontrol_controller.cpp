#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>
#include <string>
#include <Eigen/Geometry>

#include <forcecontrol/utilities.h>


typedef std::chrono::high_resolution_clock Clock;


using namespace std;

ForceControlController::ForceControlController()
{
  _pose_set        = new float[7];
  _force_set       = new float[6];
  _STIFFNESS       = new float[3];
  _FORCE_SELECTION = new int[3];
  _COMP1_K         = new float[3];
  _COMP1_ZERO      = new float[3];
  _COMP1_POLE      = new float[3];
  _COMP2_K         = new float[3];
  _COMP2_ZERO      = new float[3];
  _COMP2_POLE      = new float[3];

  _pose_offset  = new float[3]{0};
  _force_err    = new float[3]{0};
  _force_err_I  = new float[3]{0};
  _C1X          = new float[6]{0};
  _C1Y          = new float[6]{0};
  _C2X          = new float[6]{0};
  _C2Y          = new float[6]{0};
  _pose_command = new float[7];
}

ForceControlController::~ForceControlController()
{
  delete [] _pose_set;
  delete [] _force_set;
  delete [] _STIFFNESS;
  delete [] _FORCE_SELECTION;
  delete [] _COMP1_K;
  delete [] _COMP2_K;
  delete [] _COMP1_POLE;
  delete [] _COMP2_POLE;
  delete [] _COMP1_ZERO;
  delete [] _COMP2_ZERO;
  delete [] _pose_offset;
  delete [] _force_err;
  delete [] _force_err_I;
  delete [] _C1X;
  delete [] _C1Y;
  delete [] _C2X;
  delete [] _C2Y;
  delete [] _pose_command;

  if (_print_flag)
    _file.close();
}

bool ForceControlController::init(ros::NodeHandle& root_nh, ForceControlHardware* hw, std::chrono::high_resolution_clock::time_point time0)
{
  _hw    = hw;
  _time0 = time0;

  // read set pose
  float wrench[6];
  _hw->getState(_pose_set, wrench);
  for (int i = 0; i < 6; ++i) _force_set[i] = 0;
  cout << "[ForceControlController] set pose: " << endl;
  UT::stream_array_in(cout, _pose_set, 7);
  cout << endl;
  cout << "[ForceControlController] set force: " << endl;
  UT::stream_array_in(cout, _force_set, 6);
  cout << endl;

  // read controller from parameter server
  float AC_para_mass[3], AC_para_alpha[3];
  float fHz;
  float FC_PGain, FC_IGain, FC_DGain;
  string fullpath;
  root_nh.param(string("/main_loop_rate"), fHz, 500.0f);
  root_nh.param(string("/force_fb_selection/x"), _FORCE_SELECTION[0], 1);
  root_nh.param(string("/force_fb_selection/y"), _FORCE_SELECTION[1], 1);
  root_nh.param(string("/force_fb_selection/z"), _FORCE_SELECTION[2], 1);
  root_nh.param(string("/AC_para_X/k"), _STIFFNESS[0], 0.0f);
  root_nh.param(string("/AC_para_Y/k"), _STIFFNESS[1], 0.0f);
  root_nh.param(string("/AC_para_Z/k"), _STIFFNESS[2], 0.0f);
  root_nh.param(string("/AC_para_X/m"),     AC_para_mass[0], 1.0f);
  root_nh.param(string("/AC_para_Y/m"),     AC_para_mass[1], 1.0f);
  root_nh.param(string("/AC_para_Z/m"),     AC_para_mass[2], 1.0f);
  root_nh.param(string("/AC_para_X/alpha"),     AC_para_alpha[0], 100.0f);
  root_nh.param(string("/AC_para_Y/alpha"),     AC_para_alpha[1], 100.0f);
  root_nh.param(string("/AC_para_Z/alpha"),     AC_para_alpha[2], 100.0f);
  root_nh.param(string("/AC_limit"),     _COMP2_LIMIT, 100.0f);
  root_nh.param(string("/FC_gains/PGain"), _FC_PGain, 1.0f);
  root_nh.param(string("/FC_gains/IGain"), _FC_IGain, 0.0f);
  root_nh.param(string("/FC_gains/DGain"), _FC_DGain, 0.0f);
  root_nh.param(string("/FC_I_Limit"), _FC_I_Limit, 10.0f);

  root_nh.param(string("/forcecontrol_print_flag"), _print_flag, false);
  root_nh.param(string("/forcecontrol_file_path"), fullpath, string(" "));

  if (!root_nh.hasParam("/force_fb_selection"))
    ROS_WARN_STREAM("Parameter [/force_fb_selection] not found, using default: ");
  else
    ROS_INFO_STREAM("Parameter [/force_fb_selection] = ");
  ROS_INFO_STREAM(_FORCE_SELECTION[0] << "\t" <<
                  _FORCE_SELECTION[1] << "\t" <<
                  _FORCE_SELECTION[2]);

  if (!root_nh.hasParam("/AC_para_X"))
    ROS_WARN_STREAM("Parameter [/AC_para_X] not found, using default: ");
  else
    ROS_INFO_STREAM("Parameter [/AC_para_X] = ");
  ROS_INFO_STREAM(_STIFFNESS[0] << "\t" <<
                  AC_para_mass[0] << "\t" <<
                  AC_para_alpha[0]);
  if (!root_nh.hasParam("/AC_para_Y"))
    ROS_WARN_STREAM("Parameter [/AC_para_Y] not found, using default: ");
  else
    ROS_INFO_STREAM("Parameter [/AC_para_Y] = ");
  ROS_INFO_STREAM(_STIFFNESS[1] << "\t" <<
                  AC_para_mass[1] << "\t" <<
                  AC_para_alpha[1]);
  if (!root_nh.hasParam("/AC_para_Z"))
    ROS_WARN_STREAM("Parameter [/AC_para_Z] not found, using default: ");
  else
    ROS_INFO_STREAM("Parameter [/AC_para_Z] = ");
  ROS_INFO_STREAM(_STIFFNESS[2] << "\t" <<
                  AC_para_mass[2] << "\t" <<
                  AC_para_alpha[2]);

  if (!root_nh.hasParam("/AC_limit"))
    ROS_WARN_STREAM("Parameter [/AC_limit] not found, using default: " << _COMP2_LIMIT);
  else
    ROS_INFO_STREAM("Parameter [/AC_limit] = " << _COMP2_LIMIT);

  if (!root_nh.hasParam("/forcecontrol_print_flag"))
    ROS_WARN_STREAM("Parameter [/forcecontrol_print_flag] not found, using default: " << _print_flag);
  else
    ROS_INFO_STREAM("Parameter [/forcecontrol_print_flag] = " << _print_flag);

  if (!root_nh.hasParam("/forcecontrol_file_path"))
    ROS_WARN_STREAM("Parameter [/forcecontrol_file_path] not found, using default: " << fullpath);
  else
    ROS_INFO_STREAM("Parameter [/forcecontrol_file_path] = " << fullpath);

  // compute the gains
  for (int i = 0; i < 3; ++i)
  {
    _COMP1_K[i]    = 1.0f/fHz/AC_para_mass[i];
    _COMP1_ZERO[i] = 0.0f;
    _COMP1_POLE[i] = 1.0f - AC_para_alpha[i]/fHz/AC_para_mass[i];
    _COMP2_K[i]    = 1000.0f/fHz;
    _COMP2_ZERO[i] = 0.0f;
    _COMP2_POLE[i] = 1.0f;
  }

  // open file
  if (_print_flag)
  {
    _file.open(fullpath);
    if (_file.is_open())
      ROS_INFO_STREAM("[ForceControlController] file opened successfully." << endl);
    else
      ROS_ERROR_STREAM("[ForceControlController] Failed to open file." << endl);
  }

  return true;
}

void ForceControlController::setPose(const float *pose)
{
  UT::copyArray(pose, _pose_set, 7);
}

void ForceControlController::setForce(const float *force)
{
  UT::copyArray(force, _force_set, 6);
}

/*
 *
    force control law
 *
 */
void ForceControlController::update(const ros::Time& time, const ros::Duration& period)
{
  using namespace Eigen;

  float pose_fb[7];
  float wrench_fb[6];
  static Vector3f force;
  _hw->getState(pose_fb, wrench_fb);

  // ----------------------------------------
  //  Pose error
  // ----------------------------------------
  float pose_err[7];
  for (int i = 0; i < 3; ++i) pose_err[i] = _pose_set[i] - _pose_offset[i] - pose_fb[i];

  // ----------------------------------------
  //  Spring forces (using stiffness)
  // ----------------------------------------
  pose_err[0] *= _STIFFNESS[0];
  pose_err[1] *= _STIFFNESS[1];
  pose_err[2] *= _STIFFNESS[2];

  // ----------------------------------------
  //  Force feedback
  // ----------------------------------------
  static Quaternionf qn; // current orientation of end effector (also FT sensor)

  force(0) = wrench_fb[0];
  force(1) = wrench_fb[1];
  force(2) = wrench_fb[2];

  qn.w() = pose_fb[3];
  qn.x() = pose_fb[4];
  qn.y() = pose_fb[5];
  qn.z() = pose_fb[6];

  // transformation
  force = qn._transformVector(force);

  // ----------------------------------------
  //  Force error, PID force control
  // ----------------------------------------
  float force_err[3];
  force_err[0] = _force_set[0] - force(0);
  force_err[1] = _force_set[1] - force(1);
  force_err[2] = _force_set[2] - force(2);

  _force_err_I[0] += force_err[0];
  _force_err_I[1] += force_err[1];
  _force_err_I[2] += force_err[2];

  UT::truncate(_force_err_I, _FC_I_Limit, -_FC_I_Limit, 3);

  force(0) += _force_set[0] - _FC_PGain*force_err[0] - _FC_IGain*_force_err_I[0] + _FC_DGain*(force_err[0] - _force_err[0]);
  force(1) += _force_set[1] - _FC_PGain*force_err[1] - _FC_IGain*_force_err_I[1] + _FC_DGain*(force_err[1] - _force_err[1]);
  force(2) += _force_set[2] - _FC_PGain*force_err[2] - _FC_IGain*_force_err_I[2] + _FC_DGain*(force_err[2] - _force_err[2]);

  _force_err[0] = force_err[0];
  _force_err[1] = force_err[1];
  _force_err[2] = force_err[2];

  // selection
  force(0) = _FORCE_SELECTION[0] * force(0);
  force(1) = _FORCE_SELECTION[1] * force(1);
  force(2) = _FORCE_SELECTION[2] * force(2);


  pose_err[0] = pose_err[0] + force(0);
  pose_err[1] = pose_err[1] + force(1);
  pose_err[2] = pose_err[2] + force(2);


  // ----------------------------------------
  //  Compensator 1
  // ----------------------------------------
  _C1Y[0] = _COMP1_POLE[0]*_C1Y[0] + _COMP1_K[0]*pose_err[0] - _COMP1_ZERO[0]*_COMP1_K[0]*_C1X[0];
  _C1Y[1] = _COMP1_POLE[1]*_C1Y[1] + _COMP1_K[1]*pose_err[1] - _COMP1_ZERO[1]*_COMP1_K[1]*_C1X[1];
  _C1Y[2] = _COMP1_POLE[2]*_C1Y[2] + _COMP1_K[2]*pose_err[2] - _COMP1_ZERO[2]*_COMP1_K[2]*_C1X[2];
  UT::copyArray(pose_err, _C1X, 3);

  // ----------------------------------------
  //  Compensator 2
  // ----------------------------------------
  _C2Y[0] = _COMP2_POLE[0]*_C2Y[0] + _COMP2_K[0]*_C1Y[0] - _COMP2_ZERO[0]*_COMP2_K[0]*_C2X[0];
  _C2Y[1] = _COMP2_POLE[1]*_C2Y[1] + _COMP2_K[1]*_C1Y[1] - _COMP2_ZERO[1]*_COMP2_K[1]*_C2X[1];
  _C2Y[2] = _COMP2_POLE[2]*_C2Y[2] + _COMP2_K[2]*_C1Y[2] - _COMP2_ZERO[2]*_COMP2_K[2]*_C2X[2];
  UT::truncate(_C2Y, _COMP2_LIMIT, -_COMP2_LIMIT, 3);
  UT::copyArray(_C1Y, _C2X, 3);

  // ----------------------------------------
  //  Pose offset
  // ----------------------------------------
  // UT::copyArray(pose_fb, pose_command, 7);
  for (int i = 0; i < 3; ++i) _pose_command[i] = _pose_set[i] - _pose_offset[i] + _C2Y[i];
  for (int i = 3; i < 7; ++i) _pose_command[i] = _pose_set[i];

  Clock::time_point timenow_clock = Clock::now();
  double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - _time0).count())/1e6; // milli second

  _hw->setControl(_pose_command);

  // cout << "[ForceControlController] Update at "  << timenow << endl;
  if(_print_flag)
  {
    _file << timenow << " ";
    UT::stream_array_in(_file, _pose_set, 7);
    UT::stream_array_in(_file, pose_fb, 7);
    UT::stream_array_in(_file, wrench_fb, 6);
    UT::stream_array_in(_file, _C1X, 3);
    UT::stream_array_in(_file, _C1Y, 3);
    UT::stream_array_in(_file, _C2Y, 3);
    UT::stream_array_in(_file, _pose_command, 7);
    _file << endl;
  }

}


void ForceControlController::updateAxis(int *force_selection)
{
  for (int ax = 0; ax < 3; ++ax)
  {
    if (force_selection[ax] == _FORCE_SELECTION[ax])
      continue;

    if (_FORCE_SELECTION[ax])
    {
      /* force become position */
      // 1. compute offset
      float pose_fb[7];
      _hw->getPose(pose_fb);
      _pose_offset[ax] = _pose_set[ax] - _pose_command[ax];
      // // 2. reset state variables
      _C1X[ax] = 0;
      _C1Y[ax] = 0;
      _C2X[ax] = 0;
      _C2Y[ax] = 0;
    }
    else
    {
      /* position become force */
      // reset force PID states
      _force_err[ax] = 0;
      _force_err_I[ax] = 0;
    }

    _FORCE_SELECTION[ax] = force_selection[ax];
  }
}

void ForceControlController::reset()
{
  for (int ax = 0; ax < 3; ++ax)
  {
    _pose_offset[ax]  = 0;
    _force_err[ax]    = 0;
    _force_err_I[ax]  = 0;
  }
  for (int i = 0; i < 6; ++i)
  {
    _C1X[i]       = 0;
    _C1Y[i]       = 0;
    _C2X[i]       = 0;
    _C2Y[i]       = 0;
    _force_set[i] = 0;
  }
  _hw->getPose(_pose_command);
  UT::copyArray(_pose_command, _pose_set, 7);
}
