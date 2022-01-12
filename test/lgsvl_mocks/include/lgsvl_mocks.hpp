// Copyright (c) 2021-2022 Gaia Platform LLC
// SPDX-License-Identifier: MIT

#ifndef LGSVL_MOCKS_HPP_
#define LGSVL_MOCKS_HPP_

#include <string>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "lgsvl_msgs/msg/bounding_box2_d.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"

#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp"

#include "iac_msgs/msg/engine_report.hpp"
#include "iac_msgs/msg/vehicle_status.hpp"
#include "iac_msgs/msg/vehicle_command.hpp"
#include "iac_msgs/msg/misc_report.hpp"
#include "iac_msgs/msg/vehicle_manual_control_command.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/rawimu.hpp"

#include "std_msgs/msg/bool.hpp"

#include "iac_common/race_control.hpp"

using lgsvl_msgs::msg::BoundingBox2D;
using lgsvl_msgs::msg::CanBusData;

using raptor_dbw_msgs::msg::WheelSpeedReport;
using raptor_dbw_msgs::msg::AcceleratorPedalCmd;

using iac_msgs::msg::EngineReport;
using iac_msgs::msg::VehicleStatus;
using iac_msgs::msg::VehicleCommand;
using iac_msgs::msg::MiscReport;

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::Imu;
using novatel_oem7_msgs::msg::BESTVEL;
using novatel_oem7_msgs::msg::BESTPOS;
using novatel_oem7_msgs::msg::HEADING2;
using novatel_oem7_msgs::msg::RAWIMU;

using std_msgs::msg::Bool;

using indy::iac::SysState;
using indy::iac::CtState;

namespace indy
{

class LgsvlMocks : public rclcpp::Node
{
public:
  explicit LgsvlMocks(const rclcpp::NodeOptions & options);

private:
  void step_100_hz();

  void step_80_hz();

  void step_20_hz();

  void step_1_hz();

  void on_wheel_speeds_received(const BoundingBox2D::SharedPtr msg);

  void on_state_report_received(const CanBusData::SharedPtr msg);

  void on_vehicle_status_received(const VehicleStatus::SharedPtr msg);

  void on_vehicle_command_received(const VehicleCommand::SharedPtr msg);

  void on_gnss_odom_received(const Odometry::SharedPtr msg);

  void on_gps_fix_received(const NavSatFix::SharedPtr msg);

  void on_imu_received(const Imu::SharedPtr msg);

  rclcpp::Subscription<BoundingBox2D>::SharedPtr sub_wheel_speeds_;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr pub_wheel_speed_report_;

  rclcpp::Subscription<CanBusData>::SharedPtr sub_state_report_;
  rclcpp::Publisher<EngineReport>::SharedPtr pub_engine_report_;

  rclcpp::Subscription<VehicleStatus>::SharedPtr sub_vehicle_status_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr sub_vehicle_command_;
  rclcpp::Publisher<MiscReport>::SharedPtr pub_misc_report_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr pub_vehicle_command_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_gnss_odom_;
  rclcpp::Subscription<NavSatFix>::SharedPtr sub_gps_fix_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;

  rclcpp::Publisher<NavSatFix>::SharedPtr pub_top_fix_;

  rclcpp::Publisher<BESTVEL>::SharedPtr pub_top_bestvel_;
  rclcpp::Publisher<BESTPOS>::SharedPtr pub_top_bestpos_;
  rclcpp::Publisher<HEADING2>::SharedPtr pub_top_heading2_;
  rclcpp::Publisher<RAWIMU>::SharedPtr pub_top_imu_;

  rclcpp::Publisher<BESTVEL>::SharedPtr pub_bottom_bestvel_;
  rclcpp::Publisher<BESTPOS>::SharedPtr pub_bottom_bestpos_;
  rclcpp::Publisher<HEADING2>::SharedPtr pub_bottom_heading2_;
  rclcpp::Publisher<RAWIMU>::SharedPtr pub_bottom_imu_;

  rclcpp::Publisher<AcceleratorPedalCmd>::SharedPtr pub_accel_pedal_;

  int vehicle_idx_ {};
  uint8_t next_rc_sequence_number_ {};
  uint8_t switch_position_ {};
  bool publish_bestpos_ {};
  bool publish_bestvel_ {};

  SysState sys_state_ {SysState::PWR_ON};
  CtState ct_state_ {CtState::INVALID};
  Flags flags_ {};

  BoundingBox2D last_wheel_speeds_ {};
  CanBusData last_state_report_ {};
  VehicleStatus last_vehicle_status_ {};
  Odometry last_gnss_odom_ {};
  NavSatFix last_gps_fix_ {};
  Imu last_imu_ {};

  size_t current_step_ {};

  rclcpp::TimerBase::SharedPtr step_timer_100_hz_;
  rclcpp::TimerBase::SharedPtr step_timer_80_hz_;
  rclcpp::TimerBase::SharedPtr step_timer_20_hz_;
  rclcpp::TimerBase::SharedPtr step_timer_10_hz_;
  rclcpp::TimerBase::SharedPtr step_timer_1_hz_;
};

}  // namespace indy

#endif  // LGSVL_MOCKS_HPP_
