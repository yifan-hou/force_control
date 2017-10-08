#include <forcecontrol/forcecontrol_hardware.h>


class ForceControlController
{
public:
  bool init(ros::NodeHandle& root_nh, ForceControlHardware* hw);
  void ForceControlController::setPose(const float *pose);
  void update(const ros::Time& time, const ros::Duration& period);

  float *_pose_set;

  // parameters
  float _PGAIN_X;
  float _PGAIN_Y;
  float _PGAIN_Z;

private:
  ForceControlHardware *_hw;

};

