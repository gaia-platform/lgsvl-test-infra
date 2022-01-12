// Copyright (c) 2021-2022 Gaia Platform LLC
// SPDX-License-Identifier: MIT

#ifndef IAC_COMMON__RACE_CONTROL_HPP_
#define IAC_COMMON__RACE_CONTROL_HPP_

#include <cstdint>

#include "iac_msgs/msg/vehicle_bitfields.hpp"
#include "iac_msgs/msg/vehicle_command.hpp"
#include "iac_msgs/msg/vehicle_status.hpp"

#include "iac_common/detail/bitmask_operators.hpp"

using iac_msgs::msg::VehicleBitfields;
using iac_msgs::msg::VehicleCommand;
using iac_msgs::msg::VehicleStatus;

namespace indy
{

enum class FlagsBitfield : uint8_t
{
  NONE = 0,
  PURPLE = (1 << VehicleBitfields::FLAGS_PURPLE_OFFSET),
  ORANGE = (1 << VehicleBitfields::FLAGS_ORANGE_OFFSET),
  RED = (1 << VehicleBitfields::FLAGS_RED_OFFSET),
  YELLOW = (1 << VehicleBitfields::FLAGS_YELLOW_OFFSET),
  GREEN = (1 << VehicleBitfields::FLAGS_GREEN_OFFSET),
  BLUE = (1 << VehicleBitfields::FLAGS_BLUE_OFFSET),
  CHECKERED = (1 << VehicleBitfields::FLAGS_CHECKERED_OFFSET),
  BLACK = (1 << VehicleBitfields::FLAGS_BLACK_OFFSET)
};

enum class PositionCommandBitfield : uint8_t
{
  NONE = 0,
  EXIT_PIT = (1 << VehicleBitfields::POSITION_CMD_EXIT_PIT_OFFSET),
  WARMUP = (1 << VehicleBitfields::POSITION_CMD_WARMUP_OFFSET),
  FORMUP = (1 << VehicleBitfields::POSITION_CMD_FORMUP_OFFSET),
  GO_INSIDE = (1 << VehicleBitfields::POSITION_CMD_GO_INSIDE_OFFSET),
  GO_OUTSIDE = (1 << VehicleBitfields::POSITION_CMD_GO_OUTSIDE_OFFSET),
  OVERTAKE_UNDER_CAUTION = (1 << VehicleBitfields::POSITION_CMD_OVERTAKE_UNDER_CAUTION_OFFSET),
  ROLLING_START = (1 << VehicleBitfields::POSITION_CMD_ROLLING_START_OFFSET),
};

enum class LocationBitfield : uint8_t
{
  NONE = 0,
  IN_PIT_BOX_AREA = (1 << VehicleBitfields::LOCATION_IN_PIT_BOX_AREA_OFFSET),
  IN_PIT_LANE = (1 << VehicleBitfields::LOCATION_IN_PIT_LANE_OFFSET),
  EXITING_PITS = (1 << VehicleBitfields::LOCATION_EXITING_PITS_OFFSET),
  ENTERING_PITS = (1 << VehicleBitfields::LOCATION_ENTERING_PITS_OFFSET),
  ON_TRACK = (1 << VehicleBitfields::LOCATION_ON_TRACK_OFFSET),
  LAST_LAP = (1 << VehicleBitfields::LOCATION_LAST_LAP_OFFSET),
  LAST_LAP_FINISHED = (1 << VehicleBitfields::LOCATION_LAST_LAP_FINISHED_OFFSET),
  IN_GRID_BOX_AREA = (1 << VehicleBitfields::LOCATION_IN_GRID_BOX_AREA_OFFSET)
};

template<>
struct enable_bitmask_operators<FlagsBitfield>
{
  static constexpr bool enable = true;
};

template<>
struct enable_bitmask_operators<PositionCommandBitfield>
{
  static constexpr bool enable = true;
};

template<>
struct enable_bitmask_operators<LocationBitfield>
{
  static constexpr bool enable = true;
};

template<typename E>
inline bool is_any_bit_set(E const & bitfield, E const & mask)
{
  typedef typename std::underlying_type<E>::type underlying;
  return static_cast<underlying>(bitfield & mask) != 0U;
}

template<typename E>
inline bool is_bit_set(E const & bitfield, E const & mask)
{
  return (bitfield & mask) == mask;
}

template<typename E>
inline void set_bit(E & bitfield, E const & mask)
{
  bitfield |= mask;
}

template<typename E>
inline void clear_bit(E & bitfield, E const & mask)
{
  bitfield &= ~mask;
}

template<typename E>
inline void set_bit_to(E & bitfield, E const & mask, bool is_set)
{
  if (is_set) {
    set_bit(bitfield, mask);
  } else {
    clear_bit(bitfield, mask);
  }
}

class Flags
{
public:
  Flags();

  explicit Flags(FlagsBitfield const & flags);

  operator FlagsBitfield() const
  {
    return flags_;
  }

  void set_flag(FlagsBitfield flags);

  void clear_flag(FlagsBitfield flags);

  void set_flag_to(FlagsBitfield flags, bool is_active);

  bool has_any_flag(FlagsBitfield flags) const;

  bool has_flag(FlagsBitfield flags) const;

  bool has_black_flag() const;

  bool has_purple_flag() const;

  bool has_checkered_flag() const;

  bool has_red_flag() const;

  bool has_orange_flag() const;

  bool has_yellow_flag() const;

  bool has_green_flag() const;

  bool has_stopped_flag() const;

private:
  FlagsBitfield flags_;
};

void set_flags(VehicleCommand & msg, Flags const & flags);

Flags get_flags(VehicleCommand const & msg);

Flags get_flags_received(VehicleStatus const & msg);

void set_flags_received(VehicleStatus & msg, Flags const & flags);

namespace iac
{

enum class CtState : uint8_t
{
  UNINITIALIZED = 0,
  PWR_ON = 1,
  INITIALIZED = 2,
  ACT_TEST = 3,
  CRANK_READY = 4,
  CRANKING = 5,
  RACE_READY = 6,
  INIT_DRIVING = 7,
  CAUTION = 8,
  NOM_RACE = 9,
  COORD_STOP = 10,
  CONTROLLED_SHUTDOWN = 11,
  EMERGENCY_SHUTDOWN = 12,
  INVALID = 255
};

enum class SysState : uint8_t
{
  NONE = 0,
  PWR_ON = 1,
  SUBSYS_CON = 2,
  ACT_TESTING = 3,
  ACT_TEST_PASSED = 4,
  WAIT_TO_CRANK = 5,
  CRANK_CHECKS_PASSED = 6,
  CRANKING = 7,
  ENG_IDLE = 8,
  DRIVING = 9,
  SHUT_ENG = 10,
  PWR_OFF = 11,
  RAPTOR_OFF = 12,
  CRANK_CHECK_INIT = 13,
  ACT_TEST_FAIL = 14,
  CRANK_CHECKS_FAIL = 15,
  EMERGENCY_SHUTDOWN = 16,
  ENG_FAILED_TO_START = 17,
  CRITICAL_LIMIT_EXCEEDED = 18,
  INVALID = 255
};

enum class TrackCondition : uint8_t
{
  INVALID = 0,
  RED = 1,
  ORANGE = 2,
  YELLOW = 3,
  GREEN = 4
};

enum class VehicleSignal : uint8_t
{
  INVALID = 0,
  NONE = 1,
  BLACK = 2,
  CHECKERED = 4,
  PURPLE = 8,
};

TrackCondition get_track_condition_from_flags(Flags const & flags);

VehicleSignal get_vehicle_signal_from_flags(Flags const & flags);

Flags get_flags_from_track_condition(uint8_t track_condition);

Flags get_flags_from_vehicle_signal(uint8_t vehicle_signal);

}  // namespace iac

}  // namespace indy

#endif  // IAC_COMMON__RACE_CONTROL_HPP_
