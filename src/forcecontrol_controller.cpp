#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>

#include <yifanlibrary/utilities.h>

using namespace std;

ForceControlController::ForceControlController()
{
  _pose_set = new float[7];
}

ForceControlController::~ForceControlController()
{
  delete [] _pose_set;
}

bool ForceControlController::init(ros::NodeHandle& root_nh, ForceControlHardware* hw)
{
  _hw = hw;

  // read set pose
  float wrench[6];
  _hw->getState(_pose_set, wrench);
  
  // read controller from parameter server
  root_nh.param(string("/forcecontrol_PGAIN_X"), _PGAIN_X, 0);
  if (!root_nh.hasParam("/forcecontrol_PGAIN_X"))
    ROS_WARN_STREAM("Parameter [/forcecontrol_PGAIN_X] not found, using default: " << _PGAIN_X);
  else
    ROS_INFO_STREAM("Parameter [/forcecontrol_PGAIN_X] = " << _PGAIN_X);

  root_nh.param(string("/forcecontrol_PGAIN_Y"), _PGAIN_Y, 0);
  if (!root_nh.hasParam("/forcecontrol_PGAIN_Y"))
    ROS_WARN_STREAM("Parameter [/forcecontrol_PGAIN_Y] not found, using default: " << _PGAIN_Y);
  else
    ROS_INFO_STREAM("Parameter [/forcecontrol_PGAIN_Y] = " << _PGAIN_Y);

  root_nh.param(string("/forcecontrol_PGAIN_Z"), _PGAIN_Z, 0);
  if (!root_nh.hasParam("/forcecontrol_PGAIN_Z"))
    ROS_WARN_STREAM("Parameter [/forcecontrol_PGAIN_Z] not found, using default: " << _PGAIN_Z);
  else
    ROS_INFO_STREAM("Parameter [/forcecontrol_PGAIN_Z] = " << _PGAIN_Z);
  
  return true;
}

void ForceControlController::setPose(const float *pose)
{
  YF::copyArray(pose, _pose_set, 7);  
}

void ForceControlController::update(const ros::Time& time, const ros::Duration& period)
{
  float pose_fb[7];
  float wrench_fb[6];
  _hw->getState(pose_fb, wrench_fb);
  cout << "[ForceControlController] Update."  << endl;
  cout << "Pose: ";
  YF::stream_array_in(cout, pose_fb, 7);
  cout << ",\nWrench: ";
  YF::stream_array_in(cout, wrench_fb, 6);
  cout << endl;

  /* 
   *
      force control law
   *
   */

  // ----------------------------------------
  //  Pose error
  // ----------------------------------------

  float pose_err[7];
  for (int i = 0; i < 7; ++i) pose_err[i] = _pose_set[i] - pose_fb[i];

  // ----------------------------------------
  //  Gain
  // ----------------------------------------
  pose_err[0] *= _PGAIN_X;
  pose_err[1] *= _PGAIN_Y;
  pose_err[2] *= _PGAIN_Z;

  YF::copyArray(pose_fb, pose_set, 7);  

  _hw->setControl(pose_set);
}

