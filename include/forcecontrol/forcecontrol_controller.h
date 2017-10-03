#include <forcecontrol/forcecontrol_hardware.h>


class ForceControlController
{
public:
  bool init(ForceControlHardware* hw);
  void update(const ros::Time& time, const ros::Duration& period);

private:
  ForceControlHardware *_hw;

};

