#include <forcecontrol/forcecontrol_controller.h>

using namespace controller_ns;

bool forcecontrol_controller::init(ForceControlHardware* hw, ros::NodeHandle &nh)
{
  // get parameters from parameter sever
  ros::NodeHandle robot_hw_nh;

  int portnum;
  double speed_limit_tran;
  double speed_limit_rot;
  EGMControlMode mode;
  int mode_int;

  robot_hw_nh.param(std::string("egm_portnum"), portnum, 6510);
  robot_hw_nh.param(std::string("egm_speed_limit_tran"), speed_limit_tran, 0.0);
  robot_hw_nh.param(std::string("egm_speed_limit_rot"), speed_limit_rot, 0.0);
  robot_hw_nh.param(std::string("egm_mode"), mode_int, 0);
  if (!robot_hw_nh.hasParam("egm_portnum"))
    ROS_WARN_STREAM("Parameter [~egm_portnum] not found, using default: " << portnum);
  if (!robot_hw_nh.hasParam("egm_speed_limit_tran"))
    ROS_WARN_STREAM("Parameter [~egm_speed_limit_tran] not found, using default: " << speed_limit_tran );
  if (!robot_hw_nh.hasParam("egm_speed_limit_rot"))
    ROS_WARN_STREAM("Parameter [~egm_speed_limit_rot] not found, using default: " << speed_limit_rot );
  if (!robot_hw_nh.hasParam("egm_mode"))
    ROS_WARN_STREAM("Parameter [~egm_mode] not found, using default: " << mode);

  switch(mode_int) {
      case 0 : mode = CT_MODE_POSITION;
               break;
      case 1 : mode = CT_MODE_VELOCITY;
               break;       // and exits the switch
      case 2 : mode = CT_MODE_TRANSPARENT;
               break;
  }

  _hw = hw;

  return hw->init(nh, robot_hw_nh, (unsigned short)portnum, (float)speed_limit_tran, (float)speed_limit_rot, mode);
}

void forcecontrol_controller::update(const ros::Time& time, const ros::Duration& period)
{
  float pose[7];
  float wrench[6];
  float pose_set[7] = {0};
  _hw->getState(pose, wrench);

  // fancy control law

  
  _hw->setControl(pose_set);
}

  // void starting(const ros::Time& time) { }
  // void stopping(const ros::Time& time) { }

// PLUGINLIB_DECLARE_CLASS(package_name, forcecontrol_controller, controller_ns::forcecontrol_controller, controller_interface::ControllerBase);
