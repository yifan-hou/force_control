#include <forcecontrol/ati_netft_hardware.h>

#include <forcecontrol/utilities.h>

//
// The thread Function
//
typedef std::chrono::high_resolution_clock Clock;


void* ATI_Monitor(void* pParam)
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

    Clock::time_point timenow_clock = Clock::now();
    double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - netft_hardware->_time0).count())/1e6; // milli second

    if (netft_hardware->_print_flag)
    {
      netft_hardware->_file << timenow << "\t";
      UT::stream_array_in(netft_hardware->_file, netft_hardware->_force, 3);
      UT::stream_array_in(netft_hardware->_file, netft_hardware->_torque, 3);
      netft_hardware->_file << endl;
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

    // cout << "[ATINetftHardware] spinOnce" << endl;
    ros::spinOnce();
    pub_rate.sleep();
  }


}

ATINetftHardware::ATINetftHardware() {
  _force  = new double[3];
  _torque = new double[3];
}

bool ATINetftHardware::init(ros::NodeHandle& root_nh, std::chrono::high_resolution_clock::time_point time0)
{
  using namespace hardware_interface;

  _time0 = time0;

  // Get parameters from the server
  string ip_address, sensor_name, fullpath;
  root_nh.param(string("/netft_ip_address"), ip_address, string("192.168.1.1"));
  root_nh.param(string("/netft_sensor_name"), sensor_name, string("netft"));
  root_nh.param(string("/netft_frame_id"), _frame_id, string("base_link"));
  root_nh.param(string("/netft_publish_rate"), _publish_rate, 100.0);
  root_nh.param(string("/netft_print_flag"), _print_flag, false);
  root_nh.param(string("/netft_file_path"), fullpath, string(" "));
  if (!root_nh.hasParam("/netft_ip_address"))
    ROS_WARN_STREAM("Parameter [/netft_ip_address] not found, using default: " << ip_address);
  else
    ROS_INFO_STREAM("Parameter [/netft_ip_address] = " << ip_address);

  if (!root_nh.hasParam("/netft_sensor_name"))
    ROS_WARN_STREAM("Parameter [/netft_sensor_name] not found, using default: " << sensor_name );
  else
    ROS_INFO_STREAM("Parameter [/netft_sensor_name] = " << sensor_name );

  if (!root_nh.hasParam("/netft_frame_id"))
    ROS_WARN_STREAM("Parameter [/netft_frame_id] not found, using default: " << _frame_id );
  else
    ROS_INFO_STREAM("Parameter [/netft_frame_id] = " << _frame_id );

  if (!root_nh.hasParam("/netft_publish_rate"))
    ROS_WARN_STREAM("Parameter [/netft_publish_rate] not found, using default: " << _publish_rate << " Hz");
  else
    ROS_INFO_STREAM("Parameter [/netft_publish_rate] = " << _publish_rate << " Hz");

  if (!root_nh.hasParam("/netft_print_flag"))
    ROS_WARN_STREAM("Parameter [/netft_print_flag] not found, using default: " << _print_flag);
  else
    ROS_INFO_STREAM("Parameter [/netft_print_flag] = " << _print_flag);

  if (!root_nh.hasParam("/netft_file_path"))
    ROS_WARN_STREAM("Parameter [/netft_file_path] not found, using default: " << fullpath);
  else
    ROS_INFO_STREAM("Parameter [/netft_file_path] = " << fullpath);

  _netft = auto_ptr<netft_rdt_driver::NetFTRDTDriver>(new netft_rdt_driver::NetFTRDTDriver(ip_address));

  // Setup publishers
  _pub      = root_nh.advertise<geometry_msgs::WrenchStamped>(sensor_name + "/data", 100);
  _diag_pub = root_nh.advertise<diagnostic_msgs::DiagnosticArray>(sensor_name + "/diagnostics", 2);

  // Setup service
  NetFTService service(*_netft, root_nh);

  // open file
  if (_print_flag)
  {
    _file.open(fullpath);
    if (_file.is_open())
      ROS_INFO_STREAM("[ATINetftHardware] file opened successfully." << endl);
    else
      ROS_ERROR_STREAM("[ATINetftHardware] Failed to open file." << endl);
  }

  // create thread
  int rc = pthread_create(&_thread, NULL, ATI_Monitor, this);
  if (rc){
    ROS_ERROR_STREAM("[ATI_Netft_hardware] ATI Netft Hardware initialization error: unable to create thread.\n");
    return false;
  }

  // Register interfaces:
  // force_torque_interface.registerHandle(ForceTorqueSensorHandle(sensor_name, _frame_id, _force, _torque));
  // registerInterface(&force_torque_interface);
  ROS_INFO_STREAM("[ATI_Netft_hardware] Initialized successfully.\n");
  return true;
}

void ATINetftHardware::getWrench(double *wrench)
{
  wrench[0] = _force[0];
  wrench[1] = _force[1];
  wrench[2] = _force[2];
  wrench[3] = _torque[0];
  wrench[4] = _torque[1];
  wrench[5] = _torque[2];
}

ATINetftHardware::~ATINetftHardware(){
  delete [] _force;
  delete [] _torque;
  _netft.reset();
  if (_print_flag)
    _file.close();
}
