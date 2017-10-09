#include <forcecontrol/forcecontrol_hardware.h>

#define PI 3.1415926

ForceControlHardware::ForceControlHardware() {
  _WRENCH_OFFSET = new float[6];
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
  float max_dist_tran, max_dist_rot;
  float egm_safety[6];
  bool print_flag;
  string filefullpath;
  int mode_int;
  EGMControlMode mode;

  root_nh.param(std::string("/egm_portnum"), portnum, 6510);
  root_nh.param(std::string("/egm_max_dist_tran"), max_dist_tran, float(0.0));
  root_nh.param(std::string("/egm_max_dist_rot"), max_dist_rot, float(0.0));
  root_nh.param(std::string("/egm_mode"), mode_int, 0);
  root_nh.param(std::string("/egm_print_flag"), print_flag, false);
  root_nh.param(std::string("/egm_file_path"), filefullpath, std::string(" "));
  root_nh.param(std::string("/egm_safety/xmax"), egm_safety[0], float(0.0));
  root_nh.param(std::string("/egm_safety/xmin"), egm_safety[1], float(0.0));
  root_nh.param(std::string("/egm_safety/ymax"), egm_safety[2], float(0.0));
  root_nh.param(std::string("/egm_safety/ymin"), egm_safety[3], float(0.0));
  root_nh.param(std::string("/egm_safety/zmax"), egm_safety[4], float(0.0));
  root_nh.param(std::string("/egm_safety/zmin"), egm_safety[5], float(0.0));
  root_nh.param(std::string("/ati/offset/fx"), _WRENCH_OFFSET[0], float(0.0));
  root_nh.param(std::string("/ati/offset/fy"), _WRENCH_OFFSET[1], float(0.0));
  root_nh.param(std::string("/ati/offset/fz"), _WRENCH_OFFSET[2], float(0.0));
  root_nh.param(std::string("/ati/offset/tx"), _WRENCH_OFFSET[3], float(0.0));
  root_nh.param(std::string("/ati/offset/ty"), _WRENCH_OFFSET[4], float(0.0));
  root_nh.param(std::string("/ati/offset/tz"), _WRENCH_OFFSET[5], float(0.0));

  if (!root_nh.hasParam("/egm_portnum"))
    ROS_WARN_STREAM("Parameter [/egm_portnum] not found, using default: " << portnum);
  else
    ROS_INFO_STREAM("Parameter [/egm_portnum] = " << portnum);

  if (!root_nh.hasParam("/egm_max_dist_tran"))
    ROS_WARN_STREAM("Parameter [/egm_max_dist_tran] not found, using default: " << max_dist_tran );
  else
    ROS_INFO_STREAM("Parameter [/egm_max_dist_tran] = " << max_dist_tran);

  if (!root_nh.hasParam("/egm_max_dist_rot"))
    ROS_WARN_STREAM("Parameter [/egm_max_dist_rot] not found, using default: " << max_dist_rot );
  else
    ROS_INFO_STREAM("Parameter [/egm_max_dist_rot] = " << max_dist_rot);

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

  if (!root_nh.hasParam("/ati/offset"))
    ROS_WARN_STREAM("Parameter [/ati/offset] not found, using default: " << _WRENCH_OFFSET[0]);
  else
    ROS_INFO_STREAM("Parameter [/ati/offset] = "    << _WRENCH_OFFSET[0] << "\t" 
                                                    << _WRENCH_OFFSET[1] << "\t"
                                                    << _WRENCH_OFFSET[2] << "\t"
                                                    << _WRENCH_OFFSET[3] << "\t"
                                                    << _WRENCH_OFFSET[4] << "\t"
                                                    << _WRENCH_OFFSET[5]);

  switch(mode_int) {
      case 0 : mode = CT_MODE_POSITION;
               break;
      case 1 : mode = CT_MODE_VELOCITY;
               break;       // and exits the switch
      case 2 : mode = CT_MODE_TRANSPARENT;
               break;
  }

  // initialize egm
  egm->init(timer, (unsigned short)portnum, max_dist_tran, max_dist_rot, egm_safety, mode, print_flag, filefullpath);
  // egm->SetCartesianVel((float)speed_tran, (float)speed_rot);

  // Register interfaces:
  registerInterface(this);
  return true;
}

void ForceControlHardware::getState(float *pose, float *wrench)
{
  float wrench_temp[7] = {0}; // wrench measured in tool frame
  egm->GetCartesian(pose);
  ati->getWrench(wrench_temp);
  // offset
  for (int i = 0; i < 6; ++i) wrench_temp[i] -= _WRENCH_OFFSET[i];

  // transform to tool-frame
  float ang_T = 20.0*PI/180.0;
  wrench[0]   = -wrench_temp[0]*sin(ang_T) + wrench_temp[1]*cos(ang_T);
  wrench[1]   =  wrench_temp[0]*cos(ang_T) + wrench_temp[1]*sin(ang_T);
  wrench[2]   =  wrench_temp[2];
}

void ForceControlHardware::setControl(const float *pose_set)
{
  egm->SetCartesian(pose_set);
}


ForceControlHardware::~ForceControlHardware(){
  delete ati;
  delete [] _WRENCH_OFFSET;
}
