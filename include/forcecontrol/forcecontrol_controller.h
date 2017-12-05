#include <fstream>

#include <forcecontrol/forcecontrol_hardware.h>
#include <yifanlibrary/TimerLinux.h>

class ForceControlController
{
public:
  ForceControlController();
  ~ForceControlController();
  bool init(ros::NodeHandle& root_nh, ForceControlHardware *hw, Timer *timer);
  void setPose(const float *pose);
  void setForce(const float *force);
  void update(const ros::Time& time, const ros::Duration& period);

  float *_pose_set;
  float *_force_set;

  // parameters
  float *_STIFFNESS;
  float _COMP1_K;
  float _COMP1_ZERO;
  float _COMP1_POLE;
  float _COMP2_K;
  float _COMP2_ZERO;
  float _COMP2_POLE;
  float _COMP2_LIMIT;

  // intermediate variables
  float *_C1X;
  float *_C1Y;
  float *_C2X;
  float *_C2Y;

private:
  ForceControlHardware *_hw;

  // misc
  Timer *_timer;
  ofstream _file;
  bool _print_flag;
  
};

