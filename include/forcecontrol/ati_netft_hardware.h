/*
    roscontrol hardware_interface wrapper for ati Netft RDT interface.

    Three methods to use this interface:
    1. use it as a hardware_interface. It provides force_torque_sensor_interface.
    2. read the public member _force and _torque, or call getWrench()
    3. use it as the netft_rdt_node. Messages and services are established.
*/
#pragma once
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>


#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <unistd.h>
#include <pthread.h>
#include <memory>
#include <boost/program_options.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

// ATI Netft Specific headers
// in the netft_rdt_driver package
#include "netft_rdt_driver/netft_rdt_driver.h" 
#include "netft_rdt_driver/ResetThresholdLatch.h"
#include "netft_rdt_driver/SetSoftwareBias.h"

namespace po = boost::program_options;
using namespace std;

// this class is a wrapper for the two services
class NetFTService {
public:
  NetFTService(netft_rdt_driver::NetFTRDTDriver & netft, ros::NodeHandle & nodeHandle) :
    netft(netft) {
      this->resetLatchService = nodeHandle.advertiseService("/netft/reset_threshold_latch", 
        &NetFTService::resetLatch, this);
      this->setBiasService = nodeHandle.advertiseService("/netft/set_software_bias", 
        &NetFTService::setBias, this);
         
  }

  bool resetLatch(netft_rdt_driver::ResetThresholdLatch::Request & req,
    netft_rdt_driver::ResetThresholdLatch::Response & res) {
    boost::system::error_code ec = netft.resetThresholdLatch();
    if (ec) {
      res.success = false;
      res.error = ec.message();
    } else {
      res.success = true;
    }
    return true;
  }

  bool setBias(netft_rdt_driver::SetSoftwareBias::Request & req,
    netft_rdt_driver::SetSoftwareBias::Response & res) {
    boost::system::error_code ec = netft.setSoftwareBias();
    if (ec) {
      res.success = false;
      res.error = ec.message();
    } else {
      res.success = true;
    }
    return true;
  }

private:
  ros::ServiceServer resetLatchService, setBiasService;
  netft_rdt_driver::NetFTRDTDriver & netft;
};





class ATINetftHardware : public hardware_interface::RobotHW
{
  public:
    ATINetftHardware();
    virtual ~ATINetftHardware();
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    void getWrench(float *wrench);

    double *_force;
    double *_torque;
    double _publish_rate;

    // netft
    ros::Publisher _pub;
    ros::Publisher _diag_pub;
    std::auto_ptr<netft_rdt_driver::NetFTRDTDriver> _netft;
    string _frame_id;

  private:
    hardware_interface::ForceTorqueSensorInterface force_torque_interface;

    // thread
    pthread_t _thread;


};
