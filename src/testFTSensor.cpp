/** 
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include <ros/ros.h>
#include "netft_rdt_driver/netft_rdt_driver.h"
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>
#include "netft_rdt_driver/ResetThresholdLatch.h"
#include "netft_rdt_driver/SetSoftwareBias.h"

namespace po = boost::program_options;
using namespace std;

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


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "netft_node");
  ros::NodeHandle nh, nh_private("~");
  
  // Get parameters from the server
  double publish_rate;
  string ip_address, sensor_name, frame_id;
  nh_private.param(std::string("ip_address"), ip_address, std::string("192.168.1.1"));
  nh_private.param(std::string("sensor_name"), sensor_name, std::string("netft"));
  nh_private.param(std::string("frame_id"), frame_id, std::string("base_link"));
  nh_private.param(std::string("publish_rate"), publish_rate, 100.0);
  if (!nh_private.hasParam("ip_address"))
    ROS_WARN_STREAM("Parameter [~ip_address] not found, using default: " << ip_address);
  if (!nh_private.hasParam("sensor_name"))
    ROS_WARN_STREAM("Parameter [~sensor_name] not found, using default: " << sensor_name );
  if (!nh_private.hasParam("frame_id"))
    ROS_WARN_STREAM("Parameter [~frame_id] not found, using default: " << frame_id );
  if (!nh_private.hasParam("publish_rate"))
    ROS_WARN_STREAM("Parameter [~publish_rate] not found, using default: " << publish_rate << " Hz");

  std::auto_ptr<netft_rdt_driver::NetFTRDTDriver> netft(new netft_rdt_driver::NetFTRDTDriver(ip_address));
  
  // Setup publishers
  ros::Publisher pub;
  pub = nh.advertise<geometry_msgs::WrenchStamped>(sensor_name + "/data", 100);
  ros::Rate pub_rate(publish_rate);
  geometry_msgs::WrenchStamped data;

  ros::Duration diag_pub_duration(1.0);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(sensor_name + "/diagnostics", 2);
  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time last_diag_pub_time(ros::Time::now());

  NetFTService service(*netft, nh);


  while (ros::ok())
  {
    if (netft->waitForNewData())
    {
      netft->getData(data);
      data.header.frame_id = frame_id;
      pub.publish(data);
    }
    
    ros::Time current_time(ros::Time::now());
    if ( (current_time - last_diag_pub_time) > diag_pub_duration )
    {
      diag_array.status.clear();
      netft->diagnostics(diag_status);
      diag_array.status.push_back(diag_status);
      diag_array.header.stamp = ros::Time::now();
      diag_pub.publish(diag_array);
      last_diag_pub_time = current_time;
    }
    
    ros::spinOnce();
    pub_rate.sleep();
  }
  
  return 0;
}
