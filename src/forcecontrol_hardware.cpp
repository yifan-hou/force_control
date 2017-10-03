#include <forcecontrol/forcecontrol_hardware.h>


ForceControlHardware::ForceControlHardware() {
}

bool ForceControlHardware::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  using namespace hardware_interface;

  ati = new ATINetftHardware();
  egm = EGMClass::Instance();

  // initialize ati (ati parameters are handled inside)
  ati->init(root_nh, robot_hw_nh);

  // read egm parameters from parameter server
  int portnum;
  double speed_limit_tran;
  double speed_limit_rot;
  int mode_int;
  EGMControlMode mode;

  robot_hw_nh.param(std::string("/egm_portnum"), portnum, 6510);
  robot_hw_nh.param(std::string("/egm_speed_limit_tran"), speed_limit_tran, 0.0);
  robot_hw_nh.param(std::string("/egm_speed_limit_rot"), speed_limit_rot, 0.0);
  robot_hw_nh.param(std::string("/egm_mode"), mode_int, 0);
  if (!robot_hw_nh.hasParam("/egm_portnum"))
    ROS_WARN_STREAM("Parameter [/egm_portnum] not found, using default: " << portnum);
  if (!robot_hw_nh.hasParam("/egm_speed_limit_tran"))
    ROS_WARN_STREAM("Parameter [/egm_speed_limit_tran] not found, using default: " << speed_limit_tran );
  if (!robot_hw_nh.hasParam("/egm_speed_limit_rot"))
    ROS_WARN_STREAM("Parameter [/egm_speed_limit_rot] not found, using default: " << speed_limit_rot );
  if (!robot_hw_nh.hasParam("/egm_mode"))
    ROS_WARN_STREAM("Parameter [/egm_mode] not found, using default: " << mode);

  switch(mode_int) {
      case 0 : mode = CT_MODE_POSITION;
               break;
      case 1 : mode = CT_MODE_VELOCITY;
               break;       // and exits the switch
      case 2 : mode = CT_MODE_TRANSPARENT;
               break;
  }

  // initialize egm
  egm->init((unsigned short)portnum, (float)speed_limit_tran, (float)speed_limit_rot, mode);


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
