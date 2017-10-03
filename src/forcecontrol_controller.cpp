#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>

#include <yifanlibrary/utilities.h>

using namespace std;

bool ForceControlController::init(ForceControlHardware* hw)
{
  _hw = hw;
  return true;
}

void ForceControlController::update(const ros::Time& time, const ros::Duration& period)
{
  float pose[7];
  float wrench[6];
  float pose_set[7] = {0};
  _hw->getState(pose, wrench);
  cout << "[ForceControlController] Update."  << endl;
  cout << "Pose: " << pose << ", Wrench" << wrench << endl;

  // fancy control law
  YF::copyArray(pose, pose_set, 7);  

  _hw->setControl(pose_set);
}

