#include <forcecontrol/forcecontrol_hardware.h>


ForceControlHardware::ForceControlHardware() {
}

bool ForceControlHardware::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh, 
                                unsigned short portnum, float speed_limit_tran, float speed_limit_rot, EGMControlMode mode)
{
  using namespace hardware_interface;

  ati = new ATINetftHardware();
  egm = EGMClass::Instance();

  ati->init(root_nh, robot_hw_nh);
  egm->init(portnum, speed_limit_tran, speed_limit_rot, mode);

  // Register interfaces:
  registerInterface(this);
  return true;
}

void ForceControlHardware::getState(float *pose, float *wrench)
{
  egm->GetCartesian(pose);
  ati->getWrench(wrench);
}

void ForceControlHardware::setControl(const float *pose_set)
{
  egm->SetCartesian(pose_set);
}


ForceControlHardware::~ForceControlHardware(){
  delete ati;
}
