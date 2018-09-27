#include <forcecontrol/forcecontrol_hardware.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#define PI 3.1415926

using namespace Eigen;

ForceControlHardware::ForceControlHardware() {
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
  root_nh.param(std::string("/ati/offset/fx"), _Foffset[0], float(0.0));
  root_nh.param(std::string("/ati/offset/fy"), _Foffset[1], float(0.0));
  root_nh.param(std::string("/ati/offset/fz"), _Foffset[2], float(0.0));
  root_nh.param(std::string("/ati/offset/tx"), _Toffset[0], float(0.0));
  root_nh.param(std::string("/ati/offset/ty"), _Toffset[1], float(0.0));
  root_nh.param(std::string("/ati/offset/tz"), _Toffset[2], float(0.0));
  root_nh.param(std::string("/ati/gravity/x"), _Gravity[0], float(0.0));
  root_nh.param(std::string("/ati/gravity/y"), _Gravity[1], float(0.0));
  root_nh.param(std::string("/ati/gravity/z"), _Gravity[2], float(0.0));
  root_nh.param(std::string("/ati/COM/x"), _Pcom[0], float(0.0));
  root_nh.param(std::string("/ati/COM/y"), _Pcom[1], float(0.0));
  root_nh.param(std::string("/ati/COM/z"), _Pcom[2], float(0.0));
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
    ROS_WARN_STREAM("Parameter [/ati/offset] not found, using default: " << _Foffset[0]);
  else
    ROS_INFO_STREAM("Parameter [/ati/offset] = "    << _Foffset[0] << "\t"
                                                    << _Foffset[1] << "\t"
                                                    << _Foffset[2] << "\t"
                                                    << _Toffset[0] << "\t"
                                                    << _Toffset[1] << "\t"
                                                    << _Toffset[2]);

  if (!root_nh.hasParam("/ati/gravity"))
    ROS_WARN_STREAM("Parameter [/ati/gravity] not found, using default: " << _Gravity[0]);
  else
    ROS_INFO_STREAM("Parameter [/ati/gravity] = "    << _Gravity[0] << "\t"
                                                    << _Gravity[1] << "\t"
                                                    << _Gravity[2]);
  if (!root_nh.hasParam("/ati/COM"))
    ROS_WARN_STREAM("Parameter [/ati/COM] not found, using default: " << _Pcom[0]);
  else
    ROS_INFO_STREAM("Parameter [/ati/COM] = "    << _Pcom[0] << "\t"
                                                    << _Pcom[1] << "\t"
                                                    << _Pcom[2]);

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

  // Register interfaces:
  registerInterface(this);
  return true;
}

bool ForceControlHardware::getState(float *pose, float *wrench)
{
  getPose(pose);
  return getWrench(wrench);
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

  // transform to tool-frame
  // this only works if the toolframe in ABB robot has identity orientation
  // i.e. q = 1 0 0 0
  float ang_T = -112.5*PI/180.0;
  wrench[0]   = cos(ang_T)*wrench_temp[0] - sin(ang_T)*wrench_temp[1];
  wrench[1]   = sin(ang_T)*wrench_temp[0] + cos(ang_T)*wrench_temp[1];
  wrench[2]   =  wrench_temp[2];
  wrench[3]   = cos(ang_T)*wrench_temp[3] - sin(ang_T)*wrench_temp[4];
  wrench[4]   = sin(ang_T)*wrench_temp[3] + cos(ang_T)*wrench_temp[4];
  wrench[5]   =  wrench_temp[5];

  // compensate for the weight of object
  float pose[7];
  egm->GetCartesian(pose);
  Quaternionf q(pose[3], pose[4], pose[5], pose[6]);
  Vector3f GinF = q.normalized().toRotationMatrix().transpose()*_Gravity;
  Vector3f GinT = _Pcom.cross(GinF);
  wrench[0] += _Foffset[0] - GinF[0];
  wrench[1] += _Foffset[1] - GinF[1];
  wrench[2] += _Foffset[2] - GinF[2];

  wrench[3] += _Toffset[0] - GinT[0];
  wrench[4] += _Toffset[1] - GinT[1];
  wrench[5] += _Toffset[2] - GinT[2];

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
  delete [] _WRENCH_SAFETY;
}
