#include <forcecontrol/ati_netft_hardware.h>

// 
// The thread Function
// 
void* Monitor(void* pParam)
{
  ATINetftHardware *netft_hardware = (ATINetftHardware*)pParam;

  geometry_msgs::WrenchStamped data;
  ros::Rate pub_rate(netft_hardware->_publish_rate);
  ros::Duration diag_pub_duration(1.0);

  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time last_diag_pub_time(ros::Time::now());

  while (ros::ok())
  {
    if (netft_hardware->_netft->waitForNewData())
    {
      netft_hardware->_netft->getData(data);
      // read data
      netft_hardware->_force[0]  = data.wrench.force.x;
      netft_hardware->_force[1]  = data.wrench.force.y;
      netft_hardware->_force[2]  = data.wrench.force.z;
      netft_hardware->_torque[0] = data.wrench.torque.x;
      netft_hardware->_torque[1] = data.wrench.torque.y;
      netft_hardware->_torque[2] = data.wrench.torque.z;
      data.header.frame_id       = netft_hardware->_frame_id;
      netft_hardware->_pub.publish(data);
    }
    
    ros::Time current_time(ros::Time::now());
    if ( (current_time - last_diag_pub_time) > diag_pub_duration )
    {
      diag_array.status.clear();
      netft_hardware->_netft->diagnostics(diag_status);
      diag_array.status.push_back(diag_status);
      diag_array.header.stamp = ros::Time::now();
      netft_hardware->_diag_pub.publish(diag_array);
      last_diag_pub_time = current_time;
    }
    
    ros::spinOnce();
    pub_rate.sleep();
  }
  

}




ATINetftHardware::ATINetftHardware() {
  _force  = new double[3];
  _torque = new double[3];
}

bool ATINetftHardware::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh){

  using namespace hardware_interface;

  // Get parameters from the server
  string ip_address, sensor_name;
  robot_hw_nh.param(std::string("/netft_ip_address"), ip_address, std::string("192.168.1.1"));
  robot_hw_nh.param(std::string("/netft_sensor_name"), sensor_name, std::string("netft"));
  robot_hw_nh.param(std::string("/netft_frame_id"), _frame_id, std::string("base_link"));
  robot_hw_nh.param(std::string("/netft_publish_rate"), _publish_rate, 100.0);
  if (!robot_hw_nh.hasParam("/netft_ip_address"))
    ROS_WARN_STREAM("Parameter [/netft_ip_address] not found, using default: " << ip_address);
  if (!robot_hw_nh.hasParam("/netft_sensor_name"))
    ROS_WARN_STREAM("Parameter [/netft_sensor_name] not found, using default: " << sensor_name );
  if (!robot_hw_nh.hasParam("/netft_frame_id"))
    ROS_WARN_STREAM("Parameter [/netft_frame_id] not found, using default: " << _frame_id );
  if (!robot_hw_nh.hasParam("/netft_publish_rate"))
    ROS_WARN_STREAM("Parameter [/netft_publish_rate] not found, using default: " << _publish_rate << " Hz");

  _netft = std::auto_ptr<netft_rdt_driver::NetFTRDTDriver>(new netft_rdt_driver::NetFTRDTDriver(ip_address));

  // Setup publishers
  _pub      = root_nh.advertise<geometry_msgs::WrenchStamped>(sensor_name + "/data", 100);
  _diag_pub = root_nh.advertise<diagnostic_msgs::DiagnosticArray>(sensor_name + "/diagnostics", 2);

  // Setup service
  NetFTService service(*_netft, root_nh);

  // create thread
  int rc = pthread_create(&_thread, NULL, Monitor, this);
  if (rc){
    cout << "[ATI_Netft_hardware] ATI Netft Hardware initialization error: unable to create thread.\n";
    return false;
  }

  // Register interfaces:
  // force_torque_interface.registerHandle(ForceTorqueSensorHandle(sensor_name, _frame_id, _force, _torque));
  // registerInterface(&force_torque_interface);
  cout << "[ATI_Netft_hardware] Initialized successfully.\n";
  return true;
}

void ATINetftHardware::getWrench(float *wrench)
{
  wrench[0] = float(_force[0]);
  wrench[1] = float(_force[1]);
  wrench[2] = float(_force[2]);
  wrench[3] = float(_torque[0]);
  wrench[4] = float(_torque[1]);
  wrench[5] = float(_torque[2]);
}

ATINetftHardware::~ATINetftHardware(){
  delete [] _force;
  delete [] _torque;
  _netft.reset();
}
