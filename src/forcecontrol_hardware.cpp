#include <forcecontrol/forcecontrol_hardware.h>


ForceControlHardware::ForceControlHardware() {
}

bool ForceControlHardware::init(ros::NodeHandle& root_nh, Timer *timer)
{
  using namespace hardware_interface;

  ati = new ATINetftHardware();
  egm = EGMClass::Instance();

  // initialize ati (ati parameters are handled inside)
  ati->init(root_nh, timer);

  // read egm parameters from parameter server
  int portnum;
  double speed_limit_tran, speed_tran;
  double speed_limit_rot, speed_rot;
  bool print_flag;
  string filefullpath;
  int mode_int;
  EGMControlMode mode;

  root_nh.param(std::string("/egm_portnum"), portnum, 6510);
  root_nh.param(std::string("/egm_speed_limit_tran"), speed_limit_tran, 0.0);
  root_nh.param(std::string("/egm_speed_tran"), speed_tran, 0.0);
  root_nh.param(std::string("/egm_speed_limit_rot"), speed_limit_rot, 0.0);
  root_nh.param(std::string("/egm_speed_rot"), speed_rot, 0.0);
  root_nh.param(std::string("/egm_mode"), mode_int, 0);
  root_nh.param(std::string("/egm_print_flag"), print_flag, false);
  root_nh.param(std::string("/egm_file_path"), filefullpath, std::string(" "));
  if (!root_nh.hasParam("/egm_portnum"))
    ROS_WARN_STREAM("Parameter [/egm_portnum] not found, using default: " << portnum);
  else
    ROS_INFO_STREAM("Parameter [/egm_portnum] = " << portnum);

  if (!root_nh.hasParam("/egm_speed_limit_tran"))
    ROS_WARN_STREAM("Parameter [/egm_speed_limit_tran] not found, using default: " << speed_limit_tran );
  else
    ROS_INFO_STREAM("Parameter [/egm_speed_limit_tran] = " << speed_limit_tran );

  if (!root_nh.hasParam("/egm_speed_tran"))
    ROS_WARN_STREAM("Parameter [/egm_speed_tran] not found, using default: " << speed_tran );
  else
    ROS_INFO_STREAM("Parameter [/egm_speed_tran] = " << speed_tran );

  if (!root_nh.hasParam("/egm_speed_limit_rot"))
    ROS_WARN_STREAM("Parameter [/egm_speed_limit_rot] not found, using default: " << speed_limit_rot );
  else
    ROS_INFO_STREAM("Parameter [/egm_speed_limit_rot] = " << speed_limit_rot );

  if (!root_nh.hasParam("/egm_speed_rot"))
    ROS_WARN_STREAM("Parameter [/egm_speed_rot] not found, using default: " << speed_rot );
  else
    ROS_INFO_STREAM("Parameter [/egm_speed_rot] = " << speed_rot );

  if (!root_nh.hasParam("/egm_mode"))
    ROS_WARN_STREAM("Parameter [/egm_mode] not found, using default: " << mode);
  else
    ROS_INFO_STREAM("Parameter [/egm_mode] = " << mode);

  if (!root_nh.hasParam("/egm_print_flag"))
    ROS_WARN_STREAM("Parameter [/egm_print_flag] not found, using default: " << print_flag);
  else
    ROS_INFO_STREAM("Parameter [/egm_print_flag] = " << print_flag);

  if (!root_nh.hasParam("/egm_file_path"))
    ROS_WARN_STREAM("Parameter [/egm_file_path] not found, using default: " << filefullpath);
  else
    ROS_INFO_STREAM("Parameter [/egm_file_path] = " << filefullpath);

  switch(mode_int) {
      case 0 : mode = CT_MODE_POSITION;
               break;
      case 1 : mode = CT_MODE_VELOCITY;
               break;       // and exits the switch
      case 2 : mode = CT_MODE_TRANSPARENT;
               break;
  }

  // initialize egm
  egm->init(timer, (unsigned short)portnum, (float)speed_limit_tran, (float)speed_limit_rot, mode, print_flag, filefullpath);
  egm->SetCartesianVel((float)speed_tran, (float)speed_rot);

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
