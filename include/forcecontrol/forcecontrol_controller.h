#include <controller_interface/controller.h>
// #include <pluginlib/class_list_macros.h>

#include <forcecontrol/forcecontrol_hardware.h>

namespace controller_ns{

class forcecontrol_controller : public controller_interface::Controller<ForceControlHardware>
{
public:
  bool init(ForceControlHardware* hw, ros::NodeHandle &nh);
  void update(const ros::Time& time, const ros::Duration& period);

  // void starting(const ros::Time& time) { }
  // void stopping(const ros::Time& time) { }

private:
  ForceControlHardware *_hw;
};
// PLUGINLIB_DECLARE_CLASS(package_name, forcecontrol_controller, controller_ns::forcecontrol_controller, controller_interface::ControllerBase);
}//namespace