#include <forcecontrol/forcecontrol_hardware.h>

#include <yifanlibrary/utilities.h>

#define PI 3.1415926

using namespace std;
using namespace UT;

ForceControlHardware::ForceControlHardware() {
  _WRENCH_SAFETY = new double[6];
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
  double PoseSensorTool[7];

  root_nh.param(std::string("/egm_portnum"), portnum, 6510);
  root_nh.param(std::string("/egm_max_dist_tran"), max_dist_tran, 0.0f);
  root_nh.param(std::string("/egm_max_dist_rot"), max_dist_rot, 0.0f);
  root_nh.param(std::string("/egm_safety_mode"), safety_mode_int, 0);
  root_nh.param(std::string("/egm_operation_mode"), operation_mode_int, 0);
  root_nh.param(std::string("/egm_print_flag"), print_flag, false);
  root_nh.param(std::string("/egm_file_path"), filefullpath, std::string(" "));
  root_nh.param(std::string("/egm_safety_zone/xmin"), egm_safety_zone[0], 0.0f);
  root_nh.param(std::string("/egm_safety_zone/xmax"), egm_safety_zone[1], 0.0f);
  root_nh.param(std::string("/egm_safety_zone/ymin"), egm_safety_zone[2], 0.0f);
  root_nh.param(std::string("/egm_safety_zone/ymax"), egm_safety_zone[3], 0.0f);
  root_nh.param(std::string("/egm_safety_zone/zmin"), egm_safety_zone[4], 0.0f);
  root_nh.param(std::string("/egm_safety_zone/zmax"), egm_safety_zone[5], 0.0f);
  root_nh.param(std::string("/ati/offset/fx"), _Foffset[0], 0.0);
  root_nh.param(std::string("/ati/offset/fy"), _Foffset[1], 0.0);
  root_nh.param(std::string("/ati/offset/fz"), _Foffset[2], 0.0);
  root_nh.param(std::string("/ati/offset/tx"), _Toffset[0], 0.0);
  root_nh.param(std::string("/ati/offset/ty"), _Toffset[1], 0.0);
  root_nh.param(std::string("/ati/offset/tz"), _Toffset[2], 0.0);
  root_nh.param(std::string("/ati/gravity/x"), _Gravity[0], 0.0);
  root_nh.param(std::string("/ati/gravity/y"), _Gravity[1], 0.0);
  root_nh.param(std::string("/ati/gravity/z"), _Gravity[2], 0.0);
  root_nh.param(std::string("/ati/COM/x"), _Pcom[0], 0.0);
  root_nh.param(std::string("/ati/COM/y"), _Pcom[1], 0.0);
  root_nh.param(std::string("/ati/COM/z"), _Pcom[2], 0.0);
  root_nh.param(std::string("/ati/safety/fx"), _WRENCH_SAFETY[0], 0.0);
  root_nh.param(std::string("/ati/safety/fy"), _WRENCH_SAFETY[1], 0.0);
  root_nh.param(std::string("/ati/safety/fz"), _WRENCH_SAFETY[2], 0.0);
  root_nh.param(std::string("/ati/safety/tx"), _WRENCH_SAFETY[3], 0.0);
  root_nh.param(std::string("/ati/safety/ty"), _WRENCH_SAFETY[4], 0.0);
  root_nh.param(std::string("/ati/safety/tz"), _WRENCH_SAFETY[5], 0.0);
  root_nh.param(std::string("/ati/transform_sensor_to_tool/x"), PoseSensorTool[0], 0.0);
  root_nh.param(std::string("/ati/transform_sensor_to_tool/y"), PoseSensorTool[1], 0.0);
  root_nh.param(std::string("/ati/transform_sensor_to_tool/z"), PoseSensorTool[2], 0.0);
  root_nh.param(std::string("/ati/transform_sensor_to_tool/qw"), PoseSensorTool[3], 1.0);
  root_nh.param(std::string("/ati/transform_sensor_to_tool/qx"), PoseSensorTool[4], 0.0);
  root_nh.param(std::string("/ati/transform_sensor_to_tool/qy"), PoseSensorTool[5], 0.0);
  root_nh.param(std::string("/ati/transform_sensor_to_tool/qz"), PoseSensorTool[6], 0.0);

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
  if (!root_nh.hasParam("/ati/transform_sensor_to_tool"))
    ROS_WARN_STREAM("Parameter [/ati/transform_sensor_to_tool] not found, using default: " << PoseSensorTool[0]);
  else
    ROS_INFO_STREAM("Parameter [/ati/transform_sensor_to_tool] = "    << PoseSensorTool[0] << "\t"
                                                    << PoseSensorTool[1] << "\t"
                                                    << PoseSensorTool[2] << "\t"
                                                    << PoseSensorTool[3] << "\t"
                                                    << PoseSensorTool[4] << "\t"
                                                    << PoseSensorTool[5] << "\t"
                                                    << PoseSensorTool[6]);

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

  _adj_sensor_tool = SE32Adj(pose2SE3(PoseSensorTool));

  // initialize egm
  egm->init(time0, (unsigned short)portnum, max_dist_tran, max_dist_rot, egm_safety_zone, egm_safety_mode, egm_operation_mode, print_flag, filefullpath);

  // Register interfaces:
  registerInterface(this);
  return true;
}

bool ForceControlHardware::getState(double *pose, double *wrench)
{
  getPose(pose);
  return getWrench(wrench);
}

void ForceControlHardware::getPose(double *pose)
{
  float posef[7] = {0};
  egm->GetCartesian(posef);
  float2double(posef, pose, 7);
}

bool ForceControlHardware::getWrench(double *wrench)
{
  double wrench_ati[6] = {0};
  int ati_flag = ati->getWrench(wrench_ati); // wrench measured in ATI frame
  bool safety = true;
  if (ati_flag != 0) safety = false;

  Eigen::Matrix<double, 6, 1> wrench_S;
  for (int i = 0; i < 6; ++i) {
    // safety
    if(abs(wrench_ati[i]) >_WRENCH_SAFETY[i])
      safety = false;
    wrench_S[i] = wrench_ati[i];
  }

  // transform from sensor frame to tool frame
  Eigen::Matrix<double, 6, 1> wrench_T;
  wrench_T = _adj_sensor_tool.transpose() * wrench_S;

  // compensate for the weight of object
  float posef[7] = {0};
  egm->GetCartesian(posef);
  double pose[7] = {0};
  float2double(posef, pose, 7);
  Quaterniond q(pose[3], pose[4], pose[5], pose[6]);
  Vector3d GinF = q.normalized().toRotationMatrix().transpose()*_Gravity;
  Vector3d GinT = _Pcom.cross(GinF);
  wrench_T[0] += _Foffset[0] - GinF[0];
  wrench_T[1] += _Foffset[1] - GinF[1];
  wrench_T[2] += _Foffset[2] - GinF[2];

  wrench_T[3] += _Toffset[0] - GinT[0];
  wrench_T[4] += _Toffset[1] - GinT[1];
  wrench_T[5] += _Toffset[2] - GinT[2];

  for (int i = 0; i < 6; ++i) wrench[i] = wrench_T[i];

  return safety;
}


void ForceControlHardware::setControl(const double *pose_set)
{
  float posef[7] = {0};
  double2float(pose_set, posef, 7);
  egm->SetCartesian(posef);

}

ForceControlHardware::~ForceControlHardware(){
  delete ati;
  delete [] _WRENCH_SAFETY;
}
