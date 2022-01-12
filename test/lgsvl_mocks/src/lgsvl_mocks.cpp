// Copyright (c) 2021-2022 Gaia Platform LLC
// SPDX-License-Identifier: MIT

#include <memory>
#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "tf2/utils.h"

#include "iac_common/pubsub.hpp"

#include "lgsvl_mocks.hpp"

using std::chrono::duration;

namespace indy
{

LgsvlMocks::LgsvlMocks(const rclcpp::NodeOptions & options)
: rclcpp::Node("lgsvl_mocks", options)
{
  time_t t;
  std::srand((uint32_t)std::time(&t));

  // From LGSVL.
  pubsub::subscribe_from(
    this, sub_gnss_odom_, "gnss_odom",
    &LgsvlMocks::on_gnss_odom_received, rclcpp::QoS{10});
  pubsub::subscribe_from(
    this, sub_gps_fix_, "gnss_fix",
    &LgsvlMocks::on_gps_fix_received, rclcpp::QoS{10});
  pubsub::subscribe_from(
    this, sub_imu_, "imu",
    &LgsvlMocks::on_imu_received, rclcpp::QoS{10});
  pubsub::subscribe_from(
    this, sub_wheel_speeds_, "wheel_speeds",
    &LgsvlMocks::on_wheel_speeds_received, rclcpp::QoS{10});
  pubsub::subscribe_from(
    this, sub_state_report_, "state_report",
    &LgsvlMocks::on_state_report_received, rclcpp::QoS{10});

  // From car.
  pubsub::subscribe_from(
    this, sub_vehicle_status_, "ct_status",
    &LgsvlMocks::on_vehicle_status_received);

  // From race control.
  pubsub::subscribe_from(
    this, sub_vehicle_command_, "rc_to_ct",
    &LgsvlMocks::on_vehicle_command_received);

  // 100 Hz.
  pubsub::publish_to(this, pub_wheel_speed_report_, "wheel_speed_report");
  pubsub::publish_to(this, pub_engine_report_, "pt_report");
  pubsub::publish_to(this, pub_accel_pedal_, "accelerator_pedal");
  pubsub::publish_to(this, pub_top_imu_, "top_imu");
  pubsub::publish_to(this, pub_bottom_imu_, "bottom_imu");

  // 80 Hz.
  pubsub::publish_to(this, pub_top_fix_, "top_fix");

  // 20 Hz.
  pubsub::publish_to(this, pub_top_bestvel_, "top_bestvel");
  pubsub::publish_to(this, pub_top_bestpos_, "top_bestpos");
  pubsub::publish_to(this, pub_bottom_bestvel_, "bottom_bestvel");
  pubsub::publish_to(this, pub_bottom_bestpos_, "bottom_bestpos");
  pubsub::publish_to(this, pub_misc_report_, "misc_report");

  // 1 Hz.
  pubsub::publish_to(this, pub_bottom_heading2_, "bottom_heading2");
  pubsub::publish_to(this, pub_top_heading2_, "top_heading2");
  pubsub::publish_to(this, pub_vehicle_command_, "rc_to_ct");

  switch_position_ = declare_parameter("switch_position", 2);
  sys_state_ = static_cast<SysState>(declare_parameter("sys_state", 1));
  publish_bestpos_ = declare_parameter("publish_bestpos", true);
  publish_bestvel_ = declare_parameter("publish_bestvel", true);
  ct_state_ = static_cast<CtState>(declare_parameter("ct_state", 0));

  declare_parameter("track_flag", "red");
  declare_parameter("vehicle_flag", "");
  declare_parameter("vehicle_number", 1);

  vehicle_idx_ = get_parameter("vehicle_number").as_int() - 1;
  next_rc_sequence_number_ = 0;

  step_timer_100_hz_ = rclcpp::create_timer(
    this, get_clock(), duration<float>(0.01), [this] {
      step_100_hz();
    });
  step_timer_80_hz_ = rclcpp::create_timer(
    this, get_clock(), duration<float>(0.0125), [this] {
      step_80_hz();
    });
  step_timer_20_hz_ = rclcpp::create_timer(
    this, get_clock(), duration<float>(0.2), [this] {
      step_20_hz();
    });
  step_timer_1_hz_ = rclcpp::create_timer(
    this, get_clock(), duration<float>(1.0), [this] {
      step_1_hz();
    });
}

void LgsvlMocks::on_wheel_speeds_received(const BoundingBox2D::SharedPtr msg)
{
  last_wheel_speeds_ = *msg;
}

void LgsvlMocks::on_state_report_received(const CanBusData::SharedPtr msg)
{
  last_state_report_ = *msg;
}

void LgsvlMocks::on_vehicle_status_received(const VehicleStatus::SharedPtr msg)
{
  ct_state_ = static_cast<CtState>(msg->ct_state);
}

void LgsvlMocks::on_vehicle_command_received(const VehicleCommand::SharedPtr msg)
{
  flags_ = get_flags(*msg);
}

void LgsvlMocks::on_gnss_odom_received(const Odometry::SharedPtr msg)
{
  last_gnss_odom_ = *msg;
}

void LgsvlMocks::on_gps_fix_received(const NavSatFix::SharedPtr msg)
{
  last_gps_fix_ = *msg;
}

void LgsvlMocks::on_imu_received(const Imu::SharedPtr msg)
{
  last_imu_ = *msg;
}

void LgsvlMocks::step_100_hz()
{
  {
    auto msg = AcceleratorPedalCmd();
    pub_accel_pedal_->publish(msg);
  }

  {
    auto msg = WheelSpeedReport();
    msg.header.stamp = now();

    msg.front_left = last_wheel_speeds_.x;
    msg.front_right = last_wheel_speeds_.y;
    msg.rear_left = last_wheel_speeds_.width;
    msg.rear_right = last_wheel_speeds_.height;

    pub_wheel_speed_report_->publish(msg);
  }

  {
    auto msg = EngineReport();
    msg.stamp = now();

    msg.engine_rpm = last_state_report_.engine_rpm;
    msg.current_gear = last_state_report_.selected_gear;
    if (msg.current_gear == 0U) {
      msg.current_gear = 1U;
    }

    pub_engine_report_->publish(msg);
  }

  {
    auto msg = RAWIMU();
    msg.header.stamp = now();

    msg.linear_acceleration = last_imu_.linear_acceleration;
    msg.angular_velocity = last_imu_.angular_velocity;

    pub_top_imu_->publish(msg);
    pub_bottom_imu_->publish(msg);
  }

  current_step_ += 1;
}

void LgsvlMocks::step_80_hz()
{
  {
    auto msg = NavSatFix();

    msg = last_gps_fix_;
    msg.header.stamp = now();

    pub_top_fix_->publish(msg);
  }
}

void LgsvlMocks::step_20_hz()
{
  {
    auto msg = BESTVEL();
    msg.header.stamp = now();

    const auto velocity = last_gnss_odom_.twist.twist.linear;
    const auto speed =
      std::sqrt(
      std::pow(
        velocity.x,
        2.0) + std::pow(velocity.y, 2.0) + std::pow(velocity.z, 2.0));

    msg.hor_speed = speed;
    if (speed < 1.25) {
      msg.trk_gnd = static_cast<double>(std::rand() % 360);
    } else {
      auto heading = tf2::getYaw(last_gnss_odom_.pose.pose.orientation);
      heading *= -1.0;
      heading += 1.57079;

      heading = 180.0 * heading / M_PI;
      if (heading < 0.0) {
        heading += 360.0;
      } else if (heading > 360.0) {
        heading -= 360.0;
      }

      msg.trk_gnd = heading;
    }

    msg.diff_age = 0.5;
    msg.latency = 0.025;
    msg.vel_type.type = 50;
    msg.sol_status.status = 0;

    publish_bestvel_ = get_parameter("publish_bestvel").get_value<bool>();
    if (publish_bestvel_) {
      pub_top_bestvel_->publish(msg);

      if (speed < 1.25) {
        msg.trk_gnd = static_cast<double>(std::rand() % 360);
      }

      pub_bottom_bestvel_->publish(msg);
    }
  }

  {
    auto msg = BESTPOS();
    msg.header.stamp = now();

    auto svl_gps = convert_svl_gps(
      {last_gps_fix_.latitude, last_gps_fix_.longitude,
        last_gps_fix_.altitude});
    msg.lat = svl_gps.latitude;
    msg.lon = svl_gps.longitude;
    msg.hgt = svl_gps.altitude;

    msg.lat_stdev = 0.01;
    msg.lon_stdev = 0.01;
    msg.hgt_stdev = 0.01;

    msg.diff_age = 0.5;
    msg.sol_age = 0.0;
    msg.pos_type.type = 50;
    msg.sol_status.status = 0;
    msg.num_sol_svs = 5;

    publish_bestpos_ = get_parameter("publish_bestpos").get_value<bool>();
    if (publish_bestpos_) {
      pub_top_bestpos_->publish(msg);
      pub_bottom_bestpos_->publish(msg);
    }
  }

  {
    auto msg = MiscReport();
    msg.stamp = now();

    switch_position_ = get_parameter("switch_position").get_value<uint8_t>();

    if (switch_position_ == 3) {
      if (sys_state_ == SysState::PWR_ON) {
        sys_state_ = SysState::SUBSYS_CON;
      } else if (sys_state_ == SysState::SUBSYS_CON) {
        sys_state_ = SysState::ACT_TESTING;
      } else if (sys_state_ == SysState::ACT_TESTING) {
        sys_state_ = SysState::ACT_TEST_PASSED;
      }
    }

    if (switch_position_ == 4) {
      const auto can_start = ct_state_ == CtState::CRANKING && flags_.has_orange_flag();
      const auto can_drive = ct_state_ == CtState::INIT_DRIVING &&
        (flags_.has_yellow_flag() || flags_.has_green_flag());
      if (sys_state_ == SysState::ACT_TEST_PASSED && ct_state_ == CtState::ACT_TEST) {
        sys_state_ = SysState::WAIT_TO_CRANK;
      } else if (sys_state_ == SysState::WAIT_TO_CRANK && ct_state_ == CtState::CRANK_READY) {
        sys_state_ = SysState::CRANK_CHECK_INIT;
      } else if (sys_state_ == SysState::CRANK_CHECK_INIT) {
        sys_state_ = SysState::CRANK_CHECKS_PASSED;
      } else if (sys_state_ == SysState::CRANK_CHECKS_PASSED && can_start) {
        sys_state_ = SysState::CRANKING;
      } else if (sys_state_ == SysState::CRANKING) {
        sys_state_ = SysState::ENG_IDLE;
      } else if (sys_state_ == SysState::ENG_IDLE && can_drive) {
        sys_state_ = SysState::DRIVING;
      }
    }

    if (ct_state_ == CtState::EMERGENCY_SHUTDOWN) {
      sys_state_ = SysState::EMERGENCY_SHUTDOWN;
    }

    msg.sys_state = static_cast<uint8_t>(sys_state_);
    pub_misc_report_->publish(msg);
  }
}

void LgsvlMocks::step_1_hz()
{
  {
    auto msg = HEADING2();
    msg.header.stamp = now();

    auto heading = tf2::getYaw(last_gnss_odom_.pose.pose.orientation);
    heading *= -1.0;
    heading += 1.57079;

    heading = 180.0 * heading / M_PI;
    if (heading < 0.0) {
      heading += 360.0;
    } else if (heading > 360.0) {
      heading -= 360.0;
    }

    // Top NovAtel offset.
    heading += 180.0;
    if (heading > 360.0) {
      heading -= 360.0;
    }

    msg.heading = heading;

    msg.heading_stdev = 0.5;
    msg.pos_type.type = 50;
    msg.sol_status.status = 0;
    msg.num_sv_in_sol = 5;

    pub_top_heading2_->publish(msg);

    msg.heading -= 90.0;
    if (heading < 0.0) {
      heading += 360.0;
    } else if (heading > 360.0) {
      heading -= 360.0;
    }

    pub_bottom_heading2_->publish(msg);
  }

  {
    auto msg = VehicleCommand();

    msg.header.stamp = now();
    msg.header.vehicle_number = vehicle_idx_ + 1;

    msg.header.sequence_number = next_sequence_number_;
    next_sequence_number_ += 1;

    auto flags = Flags();

    auto track_flag = get_parameter("track_flag").as_string();
    if (track_flag == "red") {
      flags.set_flag(FlagsBitfield::RED);
    } else if (track_flag == "orange") {
      flags.set_flag(FlagsBitfield::ORANGE);
    } else if (track_flag == "yellow") {
      flags.set_flag(FlagsBitfield::YELLOW);
    } else if (track_flag == "green") {
      flags.set_flag(FlagsBitfield::GREEN);
    }

    auto vehicle_flag = get_parameter("vehicle_flag").as_string();
    if (vehicle_flag == "purple") {
      flags.set_flag(FlagsBitfield::PURPLE);
    } else if (vehicle_flag == "black") {
      flags.set_flag(FlagsBitfield::BLACK);
    } else if (vehicle_flag == "checkered") {
      flags.set_flag(FlagsBitfield::CHECKERED);
    }

    set_flags(msg, flags);

    pub_vehicle_command_->publish(msg);
  }
}

}  // namespace indy

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<indy::LgsvlMocks>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
