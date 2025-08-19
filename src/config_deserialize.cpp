#include "force_control/config_deserialize.h"
#include <RobotUtilities/spatial_utilities.h>

#include <yaml-cpp/yaml.h>

#include "force_control/admittance_controller.h"

template <>
bool deserialize(const YAML::Node& node,
                 AdmittanceController::AdmittanceControllerConfig& config) {
  try {
    config.dt = node["dt"].as<double>();
    config.log_to_file = node["log_to_file"].as<bool>();
    config.log_file_path = node["log_file_path"].as<std::string>();
    config.alert_overrun = node["alert_overrun"].as<bool>();
    config.compliance6d.stiffness = RUT::deserialize_vector<RUT::Vector6d>(
                                        node["compliance6d"]["stiffness"])
                                        .asDiagonal();
    config.compliance6d.damping =
        RUT::deserialize_vector<RUT::Vector6d>(node["compliance6d"]["damping"])
            .asDiagonal();
    config.compliance6d.inertia =
        RUT::deserialize_vector<RUT::Vector6d>(node["compliance6d"]["inertia"])
            .asDiagonal();
    config.compliance6d.stiction = RUT::deserialize_vector<RUT::Vector6d>(
        node["compliance6d"]["stiction"]);

    if (node["additional_damping"]) {
      config.additional_damping =
          RUT::deserialize_vector<RUT::Vector6d>(node["additional_damping"])
              .asDiagonal();
    }

    config.max_spring_force_magnitude =
        node["max_spring_force_magnitude"].as<double>();
    config.max_spring_torque_magnitude =
        node["max_spring_torque_magnitude"].as<double>();
    config.direct_force_control_gains.P_trans =
        node["direct_force_control_gains"]["P_trans"].as<double>();
    config.direct_force_control_gains.I_trans =
        node["direct_force_control_gains"]["I_trans"].as<double>();
    config.direct_force_control_gains.D_trans =
        node["direct_force_control_gains"]["D_trans"].as<double>();
    config.direct_force_control_gains.P_rot =
        node["direct_force_control_gains"]["P_rot"].as<double>();
    config.direct_force_control_gains.I_rot =
        node["direct_force_control_gains"]["I_rot"].as<double>();
    config.direct_force_control_gains.D_rot =
        node["direct_force_control_gains"]["D_rot"].as<double>();
    config.direct_force_control_I_limit =
        RUT::deserialize_vector<RUT::Vector6d>(
            node["direct_force_control_I_limit"]);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  // validity check
  if (config.dt <= 0) {
    std::cerr << "Invalid dt: " << config.dt << std::endl;
    return false;
  }
  // diagonal elements of stiffness, damping, and inertia should be positive/non-negative
  if ((config.compliance6d.stiffness.diagonal().array() < 0).any()) {
    std::cerr << "Invalid compliance6d.stiffness: "
              << config.compliance6d.stiffness.diagonal().transpose()
              << ". All diagonal elements should be positive." << std::endl;
    return false;
  }
  if ((config.compliance6d.damping.diagonal().array() < 0).any()) {
    std::cerr << "Invalid compliance6d.damping: "
              << config.compliance6d.damping.diagonal().transpose()
              << ". All diagonal elements should be positive." << std::endl;
    return false;
  }
  if ((config.compliance6d.inertia.diagonal().array() <= 0).any()) {
    std::cerr << "Invalid compliance6d.inertia: "
              << config.compliance6d.inertia.diagonal().transpose()
              << ". All diagonal elements should be positive." << std::endl;
    return false;
  }
  // stiction should be non-negative
  if ((config.compliance6d.stiction.array() < 0).any()) {
    std::cerr << "Invalid compliance6d.stiction: "
              << config.compliance6d.stiction.transpose()
              << ". All elements should be non-negative." << std::endl;
    return false;
  }
  // direct force control gains should be non-negative
  if ((config.direct_force_control_gains.P_trans < 0) ||
      (config.direct_force_control_gains.I_trans < 0) ||
      (config.direct_force_control_gains.D_trans < 0) ||
      (config.direct_force_control_gains.P_rot < 0) ||
      (config.direct_force_control_gains.I_rot < 0) ||
      (config.direct_force_control_gains.D_rot < 0)) {
    std::cerr << "Invalid direct_force_control_gains. All elements should be "
                 "non-negative."
              << std::endl;
    return false;
  }

  // direct force control gains must be all zero if stiction is not zero.
  bool has_direct_force_control_gains =
      ((config.direct_force_control_gains.P_trans > 0) ||
       (config.direct_force_control_gains.I_trans > 0) ||
       (config.direct_force_control_gains.D_trans > 0) ||
       (config.direct_force_control_gains.P_rot > 0) ||
       (config.direct_force_control_gains.I_rot > 0) ||
       (config.direct_force_control_gains.D_rot > 0));
  if ((config.compliance6d.stiction.array() > 0).any() &&
      has_direct_force_control_gains) {
    std::cerr << "Invalid parameters. direct_force_control_gains must be all "
                 "zero if stiction is non-zero."
              << std::endl;
    return false;
  }

  return true;
}