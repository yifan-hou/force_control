#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>
#include <string>
#include <Eigen/Geometry>

#include <forcecontrol/utilities.h>


typedef std::chrono::high_resolution_clock Clock;


using namespace std;

ForceControlController::ForceControlController()
{
  _pose_set = new float[7];
  _force_set = new float[6];
  _STIFFNESS = new float[6];

  _C1X = new float[6];
  _C1Y = new float[6];
  _C2X = new float[6];
  _C2Y = new float[6];
  for (int i = 0; i < 6; ++i)
  {
    _C1X[i] = 0;
    _C1Y[i] = 0;
    _C2X[i] = 0;
    _C2Y[i] = 0;
  }
}

ForceControlController::~ForceControlController()
{
  delete [] _pose_set;
  delete [] _force_set;
  delete [] _STIFFNESS;
  delete [] _C1X;
  delete [] _C1Y;
  delete [] _C2X;
  delete [] _C2Y;

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
  string fullpath;
  root_nh.param(string("/stiffness/x"), _STIFFNESS[0], float(0.0));
  root_nh.param(string("/stiffness/y"), _STIFFNESS[1], float(0.0));
  root_nh.param(string("/stiffness/z"), _STIFFNESS[2], float(0.0));
  root_nh.param(string("/comp1/k"),     _COMP1_K,      float(0.0));
  root_nh.param(string("/comp1/zero"),  _COMP1_ZERO,   float(0.0));
  root_nh.param(string("/comp1/pole"),  _COMP1_POLE,   float(0.0));
  root_nh.param(string("/comp2/k"),     _COMP2_K,      float(0.0));
  root_nh.param(string("/comp2/zero"),  _COMP2_ZERO,   float(0.0));
  root_nh.param(string("/comp2/pole"),  _COMP2_POLE,   float(0.0));
  root_nh.param(string("/comp2/limit"), _COMP2_LIMIT,  float(0.0));
  root_nh.param(string("/forcecontrol_print_flag"), _print_flag, false);
  root_nh.param(string("/forcecontrol_file_path"), fullpath, string(" "));
  
  if (!root_nh.hasParam("/stiffness"))
    ROS_WARN_STREAM("Parameter [/stiffness] not found, using default: " << _STIFFNESS[0]);
  else
    ROS_INFO_STREAM("Parameter [/stiffness] = " << _STIFFNESS[0] << "\t"
                                                << _STIFFNESS[1] << "\t"
                                                << _STIFFNESS[2]);
  if (!root_nh.hasParam("/comp1"))
    ROS_WARN_STREAM("Parameter [/comp1] not found, using default: " << _COMP1_K );
  else
    ROS_INFO_STREAM("Parameter [/comp1] = " << _COMP1_K << "\t"
                                            << _COMP1_ZERO << "\t"
                                            << _COMP1_POLE);
  if (!root_nh.hasParam("/comp2"))
    ROS_WARN_STREAM("Parameter [/comp2] not found, using default: " << _COMP2_K );
  else
    ROS_INFO_STREAM("Parameter [/comp2] = " << _COMP2_K << "\t"
                                            << _COMP2_ZERO << "\t"
                                            << _COMP2_POLE << "\t"
                                            << _COMP2_LIMIT);
  
  if (!root_nh.hasParam("/forcecontrol_print_flag"))
    ROS_WARN_STREAM("Parameter [/forcecontrol_print_flag] not found, using default: " << _print_flag);
  else
    ROS_INFO_STREAM("Parameter [/forcecontrol_print_flag] = " << _print_flag);

  if (!root_nh.hasParam("/forcecontrol_file_path"))
    ROS_WARN_STREAM("Parameter [/forcecontrol_file_path] not found, using default: " << fullpath);
  else
    ROS_INFO_STREAM("Parameter [/forcecontrol_file_path] = " << fullpath);

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
  for (int i = 0; i < 7; ++i) pose_err[i] = _pose_set[i] - pose_fb[i];

  // ----------------------------------------
  //  stiffness
  // ----------------------------------------
  pose_err[0] *= _STIFFNESS[0];
  pose_err[1] *= _STIFFNESS[1];
  pose_err[2] *= _STIFFNESS[2];

  // ----------------------------------------
  //  Force feedback
  // ----------------------------------------
  // static Quaternionf q0(float(1), float(0), float(0), float(0)); // initial orientation
  static Quaternionf qn; // current orientation of end effector (also FT sensor)

  force(0) = wrench_fb[0];
  force(1) = wrench_fb[1];
  force(2) = wrench_fb[2];

  qn.w() = pose_fb[3];
  qn.x() = pose_fb[4];
  qn.y() = pose_fb[5];
  qn.z() = pose_fb[6];

  force = qn._transformVector(force);

  // ----------------------------------------
  //  Force feedforward (offset)
  // ----------------------------------------
  force(0) += _force_set[0];
  force(1) += _force_set[1];
  force(2) += _force_set[2];

  pose_err[0] = pose_err[0] - force(0);
  pose_err[1] = pose_err[1] - force(1);
  pose_err[2] = pose_err[2] - force(2);


  // ----------------------------------------
  //  Compensator 1 
  // ----------------------------------------
  _C1Y[0] = _COMP1_POLE*_C1Y[0] + _COMP1_K*pose_err[0] - _COMP1_ZERO*_COMP1_K*_C1X[0];
  _C1Y[1] = _COMP1_POLE*_C1Y[1] + _COMP1_K*pose_err[1] - _COMP1_ZERO*_COMP1_K*_C1X[1];
  _C1Y[2] = _COMP1_POLE*_C1Y[2] + _COMP1_K*pose_err[2] - _COMP1_ZERO*_COMP1_K*_C1X[2];
  UT::copyArray(pose_err, _C1X, 3);  
  
  // ----------------------------------------
  //  Compensator 2 
  // ----------------------------------------
  _C2Y[0] = _COMP2_POLE*_C2Y[0] + _COMP2_K*_C1Y[0] - _COMP2_ZERO*_COMP2_K*_C2X[0];
  _C2Y[1] = _COMP2_POLE*_C2Y[1] + _COMP2_K*_C1Y[1] - _COMP2_ZERO*_COMP2_K*_C2X[1];
  _C2Y[2] = _COMP2_POLE*_C2Y[2] + _COMP2_K*_C1Y[2] - _COMP2_ZERO*_COMP2_K*_C2X[2];
  UT::truncate(_C2Y, _COMP2_LIMIT, -_COMP2_LIMIT, 3);
  UT::copyArray(_C1Y, _C2X, 3);  

  // ----------------------------------------
  //  Pose offset 
  // ----------------------------------------
  float pose_command[7] = {0};
  UT::copyArray(pose_fb, pose_command, 7);  
  for (int i = 0; i < 3; ++i) pose_command[i] = _pose_set[i] + _C2Y[i];
  for (int i = 3; i < 7; ++i) pose_command[i] = _pose_set[i];

  Clock::time_point timenow_clock = Clock::now();
  double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - _time0).count())/1e6; // milli second

  _hw->setControl(pose_command);

  cout << "[ForceControlController] Update at "  << timenow << endl;
  
  if(_print_flag)
  {
    _file << timenow << " ";
    UT::stream_array_in(_file, _pose_set, 7);
    UT::stream_array_in(_file, pose_fb, 7);
    UT::stream_array_in(_file, wrench_fb, 6);
    UT::stream_array_in(_file, _C1X, 3);
    UT::stream_array_in(_file, _C1Y, 3);
    UT::stream_array_in(_file, _C2Y, 3);
    UT::stream_array_in(_file, pose_command, 7);
    _file << endl;
  }

}

