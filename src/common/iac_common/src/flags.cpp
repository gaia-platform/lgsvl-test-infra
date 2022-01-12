// Copyright (c) 2021-2022 Gaia Platform LLC
// SPDX-License-Identifier: MIT

#include "iac_common/race_control.hpp"

namespace indy
{

static constexpr FlagsBitfield TRACK_CONDITIONS {
  FlagsBitfield::ORANGE |
  FlagsBitfield::RED |
  FlagsBitfield::YELLOW |
  FlagsBitfield::GREEN
};

Flags::Flags()
: flags_{FlagsBitfield::NONE}
{
}

Flags::Flags(FlagsBitfield const & flags)
: flags_{flags}
{
}

void Flags::set_flag(FlagsBitfield flags)
{
  set_bit(flags_, flags);
}

void Flags::clear_flag(FlagsBitfield flags)
{
  clear_bit(flags_, flags);
}

void Flags::set_flag_to(FlagsBitfield flags, bool is_active)
{
  set_bit_to(flags_, flags, is_active);
}

bool Flags::has_any_flag(FlagsBitfield flags) const
{
  return is_any_bit_set(flags_, flags);
}

bool Flags::has_flag(FlagsBitfield flags) const
{
  return is_bit_set(flags_, flags);
}

bool Flags::has_black_flag() const
{
  return has_flag(FlagsBitfield::BLACK);
}

bool Flags::has_purple_flag() const
{
  return has_flag(FlagsBitfield::PURPLE);
}

bool Flags::has_checkered_flag() const
{
  return has_flag(FlagsBitfield::CHECKERED);
}

bool Flags::has_stopped_flag() const
{
  return has_purple_flag() || has_red_flag() || has_orange_flag();
}

bool Flags::has_red_flag() const
{
  auto is_invalid = !has_any_flag(TRACK_CONDITIONS);
  return has_flag(FlagsBitfield::RED) || is_invalid;
}

bool Flags::has_orange_flag() const
{
  return has_flag(FlagsBitfield::ORANGE);
}

bool Flags::has_yellow_flag() const
{
  return has_flag(FlagsBitfield::YELLOW);
}

bool Flags::has_green_flag() const
{
  return has_flag(FlagsBitfield::GREEN);
}

void set_flags(VehicleCommand & msg, Flags const & flags)
{
  msg.flags = static_cast<uint8_t>(static_cast<FlagsBitfield>(flags));
}

Flags get_flags(VehicleCommand const & msg)
{
  return Flags(static_cast<FlagsBitfield>(msg.flags));
}

Flags get_flags_received(VehicleStatus const & msg)
{
  return Flags(static_cast<FlagsBitfield>(msg.flags_received));
}

void set_flags_received(VehicleStatus & msg, Flags const & flags)
{
  msg.flags_received = static_cast<uint8_t>(static_cast<FlagsBitfield>(flags));
}

namespace iac
{

TrackCondition get_track_condition_from_flags(Flags const & flags)
{
  if (flags.has_red_flag()) {
    return TrackCondition::RED;
  } else if (flags.has_orange_flag()) {
    return TrackCondition::ORANGE;
  } else if (flags.has_yellow_flag()) {
    return TrackCondition::YELLOW;
  } else if (flags.has_green_flag()) {
    return TrackCondition::GREEN;
  }

  // Treat NONE as RED.
  return TrackCondition::RED;
}

VehicleSignal get_vehicle_signal_from_flags(Flags const & flags)
{
  if (flags.has_purple_flag()) {
    return VehicleSignal::PURPLE;
  } else if (flags.has_black_flag()) {
    return VehicleSignal::BLACK;
  } else if (flags.has_checkered_flag()) {
    return VehicleSignal::CHECKERED;
  }

  return VehicleSignal::NONE;
}

Flags get_flags_from_track_condition(uint8_t track_condition)
{
  if (track_condition == static_cast<uint8_t>(TrackCondition::RED)) {
    return Flags(FlagsBitfield::RED);
  } else if (track_condition == static_cast<uint8_t>(TrackCondition::ORANGE)) {
    return Flags(FlagsBitfield::ORANGE);
  } else if (track_condition == static_cast<uint8_t>(TrackCondition::YELLOW)) {
    return Flags(FlagsBitfield::YELLOW);
  } else if (track_condition == static_cast<uint8_t>(TrackCondition::GREEN)) {
    return Flags(FlagsBitfield::GREEN);
  }

  // Treat INVALID as RED.
  return Flags(FlagsBitfield::RED);
}

Flags get_flags_from_vehicle_signal(uint8_t vehicle_signal)
{
  if (vehicle_signal == static_cast<uint8_t>(VehicleSignal::PURPLE)) {
    return Flags(FlagsBitfield::PURPLE);
  } else if (vehicle_signal == static_cast<uint8_t>(VehicleSignal::BLACK)) {
    return Flags(FlagsBitfield::BLACK);
  } else if (vehicle_signal == static_cast<uint8_t>(VehicleSignal::CHECKERED)) {
    return Flags(FlagsBitfield::CHECKERED);
  }

  return Flags(FlagsBitfield::NONE);
}

}  // namespace iac

}  // namespace indy
