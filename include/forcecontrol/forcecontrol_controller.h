#include <fstream>
#include <chrono>

#include <Eigen/Geometry>

#include <forcecontrol/forcecontrol_hardware.h>

class ForceControlController
{
public:
  ForceControlController();
  ~ForceControlController();
  bool init(ros::NodeHandle& root_nh, ForceControlHardware *hw, std::chrono::high_resolution_clock::time_point time0);
  void setPose(const float *pose);
  void setForce(const float *force);
  bool update(const ros::Time& time, const ros::Duration& period);
  void updateAxis(Eigen::Matrix3f T, int n_af);
  void reset(); // reset the state variables in the control law

  float *_pose_set;
  Eigen::Vector3f _f_Tset;

  // parameters
  Eigen::Vector3f _STIFFNESS{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _COMP1_K{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _COMP1_ZERO{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _COMP1_POLE{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _COMP2_K{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _COMP2_ZERO{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _COMP2_POLE{0.0f, 0.0f, 0.0f};
  float _kForceControlPGain, _kForceControlIGain, _kForceControlDGain;

  float _COMP2_LIMIT, _FC_I_Limit;

  // intermediate variables
  Eigen::Matrix3f _T;
  int _n_af;
  float *_pose_offset;
  float *_pose_command;
  Eigen::Vector3f _f_TErr;
  Eigen::Vector3f _f_TErr_I;
  Eigen::Vector3f _f_TAll_old{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _v_T{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _v_T_old{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _p_T{0.0f, 0.0f, 0.0f};

private:
  ForceControlHardware *_hw;

  // misc
  std::chrono::high_resolution_clock::time_point _time0; ///< high resolution timer.
  ofstream _file;
  bool _print_flag;

};

