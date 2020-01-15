#include <force_control/force_control_hardware.h>

#include <RobotUtilities/utilities.h>

#define PI 3.1415926

using namespace std;
using namespace RUT;

ForceControlHardware::ForceControlHardware() {
}

ForceControlHardware::~ForceControlHardware(){
}

bool ForceControlHardware::init(ros::NodeHandle& root_nh,
    Clock::time_point time0, FTInterfaces *ft, RobotInterfaces *robot) {
  _robot = robot;
  _ft = ft;

  return true;
}

int ForceControlHardware::getState(double *pose, double *wrench)
{
  getPose(pose);
  return getWrench(wrench);
}

void ForceControlHardware::getPose(double *pose)
{
  _robot->getCartesian(pose);
}

int ForceControlHardware::getWrench(double *wrench)
{
  double pose[7] = {0};
  _robot->getCartesian(pose);
  int safety_flag = _ft->getWrenchNetTool(pose, wrench);

  return safety_flag;
}


void ForceControlHardware::setPose(const double *pose_set)
{
  _robot->setCartesian(pose_set);

}

