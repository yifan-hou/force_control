#include <fstream>
#include <chrono>

#include <forcecontrol/forcecontrol_hardware.h>

class ForceControlController
{
public:
  ForceControlController();
  ~ForceControlController();
  bool init(ros::NodeHandle& root_nh, ForceControlHardware *hw, std::chrono::high_resolution_clock::time_point time0);
  void setPose(const float *pose);
  void setForce(const float *force);
  void update(const ros::Time& time, const ros::Duration& period);
  void updateAxis(int *force_selection);

  float *_pose_set;
  float *_force_set;

  // parameters
  float *_STIFFNESS;
  int *_FORCE_SELECTION;
  float *_COMP1_K;
  float *_COMP1_ZERO;
  float *_COMP1_POLE;
  float *_COMP2_K;
  float *_COMP2_ZERO;
  float *_COMP2_POLE;
  float _COMP2_LIMIT;

  // intermediate variables
  float *_pose_offset;
  float *_C1X;
  float *_C1Y;
  float *_C2X;
  float *_C2Y;

private:
  ForceControlHardware *_hw;

  // misc
  std::chrono::high_resolution_clock::time_point _time0; ///< high resolution timer.
  ofstream _file;
  bool _print_flag;
  
};

