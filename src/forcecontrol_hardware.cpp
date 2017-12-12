#include <forcecontrol/forcecontrol_hardware.h>

#define PI 3.1415926

ForceControlHardware::ForceControlHardware() {
  _WRENCH_OFFSET = new float[6];
  _WRENCH_SAFETY = new float[6];
}

bool ForceControlHardware::init(ros::NodeHandle& root_nh, std::chrono::high_resolution_clock::time_point time0)
{
  using namespace hardware_interface;

  ati = new ATINetftHardware();
  egm = EGMClass::Instance();

  // initialize ati (ati parameters are handled inside)
  ati->init(root_nh, time0);

  // read egm parameters from parameter server
  int portnum;
  float max_dist_tran, max_dist_rot;
  float egm_safety_zone[6];
  bool print_flag;
  string filefullpath;
  int safety_mode_int;
  int operation_mode_int;
  EGMSafetyMode egm_safety_mode;
  EGMOperationMode egm_operation_mode;

  root_nh.param(std::string("/egm_portnum"), portnum, 6510);
  root_nh.param(std::string("/egm_max_dist_tran"), max_dist_tran, float(0.0));
  root_nh.param(std::string("/egm_max_dist_rot"), max_dist_rot, float(0.0));
  root_nh.param(std::string("/egm_safety_mode"), safety_mode_int, 0);
  root_nh.param(std::string("/egm_operation_mode"), operation_mode_int, 0);
  root_nh.param(std::string("/egm_print_flag"), print_flag, false);
  root_nh.param(std::string("/egm_file_path"), filefullpath, std::string(" "));
  root_nh.param(std::string("/egm_safety_zone/xmin"), egm_safety_zone[0], float(0.0));
  root_nh.param(std::string("/egm_safety_zone/xmax"), egm_safety_zone[1], float(0.0));
  root_nh.param(std::string("/egm_safety_zone/ymin"), egm_safety_zone[2], float(0.0));
  root_nh.param(std::string("/egm_safety_zone/ymax"), egm_safety_zone[3], float(0.0));
  root_nh.param(std::string("/egm_safety_zone/zmin"), egm_safety_zone[4], float(0.0));
  root_nh.param(std::string("/egm_safety_zone/zmax"), egm_safety_zone[5], float(0.0));
  root_nh.param(std::string("/ati/offset/fx"), _WRENCH_OFFSET[0], float(0.0));
  root_nh.param(std::string("/ati/offset/fy"), _WRENCH_OFFSET[1], float(0.0));
  root_nh.param(std::string("/ati/offset/fz"), _WRENCH_OFFSET[2], float(0.0));
  root_nh.param(std::string("/ati/offset/tx"), _WRENCH_OFFSET[3], float(0.0));
  root_nh.param(std::string("/ati/offset/ty"), _WRENCH_OFFSET[4], float(0.0));
  root_nh.param(std::string("/ati/offset/tz"), _WRENCH_OFFSET[5], float(0.0));
  root_nh.param(std::string("/ati/safety/fx"), _WRENCH_SAFETY[0], float(0.0));
  root_nh.param(std::string("/ati/safety/fy"), _WRENCH_SAFETY[1], float(0.0));
  root_nh.param(std::string("/ati/safety/fz"), _WRENCH_SAFETY[2], float(0.0));
  root_nh.param(std::string("/ati/safety/tx"), _WRENCH_SAFETY[3], float(0.0));
  root_nh.param(std::string("/ati/safety/ty"), _WRENCH_SAFETY[4], float(0.0));
  root_nh.param(std::string("/ati/safety/tz"), _WRENCH_SAFETY[5], float(0.0));

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

  if (!root_nh.hasParam("/egm_safety_mode"))
    ROS_WARN_STREAM("Parameter [/egm_safety_mode] not found, using default: " << safety_mode_int);
  else
    ROS_INFO_STREAM("Parameter [/egm_safety_mode] = " << safety_mode_int);

  if (!root_nh.hasParam("/egm_operation_mode"))
    ROS_WARN_STREAM("Parameter [/egm_operation_mode] not found, using default: " << operation_mode_int);
  else
    ROS_INFO_STREAM("Parameter [/egm_operation_mode] = " << operation_mode_int);

  if (!root_nh.hasParam("/egm_print_flag"))
    ROS_WARN_STREAM("Parameter [/egm_print_flag] not found, using default: " << print_flag);
  else
    ROS_INFO_STREAM("Parameter [/egm_print_flag] = " << print_flag);

  if (!root_nh.hasParam("/egm_file_path"))
    ROS_WARN_STREAM("Parameter [/egm_file_path] not found, using default: " << filefullpath);
  else
    ROS_INFO_STREAM("Parameter [/egm_file_path] = " << filefullpath);

  if (!root_nh.hasParam("/egm_safety_zone/xmin"))
    ROS_WARN_STREAM("Parameter [/egm/egm_safety_zone] not found, using default: " << egm_safety_zone[0]);
  else
    ROS_INFO_STREAM("Parameter [/egm/egm_safety_zone] = "    << egm_safety_zone[0] << "\t" 
                                                    << egm_safety_zone[1] << "\t"
                                                    << egm_safety_zone[2] << "\t"
                                                    << egm_safety_zone[3] << "\t"
                                                    << egm_safety_zone[4] << "\t"
                                                    << egm_safety_zone[5]);

  if (!root_nh.hasParam("/ati/offset"))
    ROS_WARN_STREAM("Parameter [/ati/offset] not found, using default: " << _WRENCH_OFFSET[0]);
  else
    ROS_INFO_STREAM("Parameter [/ati/offset] = "    << _WRENCH_OFFSET[0] << "\t" 
                                                    << _WRENCH_OFFSET[1] << "\t"
                                                    << _WRENCH_OFFSET[2] << "\t"
                                                    << _WRENCH_OFFSET[3] << "\t"
                                                    << _WRENCH_OFFSET[4] << "\t"
                                                    << _WRENCH_OFFSET[5]);
  if (!root_nh.hasParam("/ati/safety"))
    ROS_WARN_STREAM("Parameter [/ati/safety] not found, using default: " << _WRENCH_SAFETY[0]);
  else
    ROS_INFO_STREAM("Parameter [/ati/safety] = "    << _WRENCH_SAFETY[0] << "\t" 
                                                    << _WRENCH_SAFETY[1] << "\t"
                                                    << _WRENCH_SAFETY[2] << "\t"
                                                    << _WRENCH_SAFETY[3] << "\t"
                                                    << _WRENCH_SAFETY[4] << "\t"
                                                    << _WRENCH_SAFETY[5]);

  switch(safety_mode_int) {
      case 0 : egm_safety_mode = SAFETY_MODE_NONE;
               break;
      case 1 : egm_safety_mode = SAFETY_MODE_TRUNCATE;
               break;
      case 2 : egm_safety_mode = SAFETY_MODE_STOP;
               break;
  }

  switch(operation_mode_int) {
      case 0 : egm_operation_mode = OPERATION_MODE_CARTESIAN;
               break;
      case 1 : egm_operation_mode = OPERATION_MODE_JOINT;
               break;
  }

  // initialize egm
  egm->init(time0, (unsigned short)portnum, max_dist_tran, max_dist_rot, egm_safety_zone, egm_safety_mode, egm_operation_mode, print_flag, filefullpath);
  // egm->SetCartesianVel((float)speed_tran, (float)speed_rot);

  // Register interfaces:
  registerInterface(this);
  return true;
}

void ForceControlHardware::getState(float *pose, float *wrench)
{
  getPose(pose);
  getWrench(wrench);
}

void ForceControlHardware::getPose(float *pose)
{
  egm->GetCartesian(pose);
}

bool ForceControlHardware::getWrench(float *wrench)
{
  float wrench_temp[7] = {0}; // wrench measured in tool frame
  ati->getWrench(wrench_temp);

  // safety
  bool safety = true;
  for (int i = 0; i < 6; ++i) 
  {
    if(abs(wrench_temp[i]) >_WRENCH_SAFETY[i])
      safety = false;
  }

  // offset
  for (int i = 0; i < 6; ++i) wrench_temp[i] -= _WRENCH_OFFSET[i];

  // transform to tool-frame
  // this only works if the toolframe in ABB robot has identity orientation
  // i.e. q = 1 0 0 0
  float ang_T = -112.5*PI/180.0;
  wrench[0]   = cos(ang_T)*wrench_temp[0] - sin(ang_T)*wrench_temp[1];
  wrench[1]   = sin(ang_T)*wrench_temp[0] + cos(ang_T)*wrench_temp[1];
  wrench[2]   =  wrench_temp[2];
  return safety;
}


void ForceControlHardware::setControl(const float *pose_set)
{
  egm->SetCartesian(pose_set);
}

void ForceControlHardware::liftup(const float dz)
{
  float pose[7];
  getPose(pose);
  pose[2] += dz;
  setControl(pose);
}

ForceControlHardware::~ForceControlHardware(){
  delete ati;
  delete [] _WRENCH_OFFSET;
  delete [] _WRENCH_SAFETY;
}
