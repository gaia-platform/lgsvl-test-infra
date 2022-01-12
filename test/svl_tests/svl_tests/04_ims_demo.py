#!/usr/bin/env python3

from . import svl_tests

import os


TEST_NAME = os.path.basename(__file__)


def test(test_env):
    # Load the map.
    spawns = test_env.load_ims()

    # Setup ego.
    ego_spawn = spawns[0]
    ego_spawn.position.x += 135.0  # NOTE(dvd): This is pit 7.
    test_env.spawn_ego_car(ego_spawn)

    steps = svl_tests.default_start_steps() + [
        svl_tests.TestStep(action=svl_tests.set_flag("yellow")),
        svl_tests.TestStep(
            trigger=lambda state: state.crossed_back_straight and state.num_laps == 0,
            action=svl_tests.set_flag("green"),
        ),
        svl_tests.TestStep(
            trigger=lambda state: state.crossed_turn_four_exit and state.num_laps == 4,
            action=svl_tests.set_flag("red"),
        ),
        svl_tests.TestStep(wait_for=5.0, action=svl_tests.setup_pylons),
        svl_tests.TestStep(wait_for=5.0, action=svl_tests.set_flag("green")),
        svl_tests.TestStep(
            trigger=lambda state: state.crossed_back_straight and state.num_laps == 5,
            action=svl_tests.set_flag("black"),
        ),
    ]

    def update_step(state, time_since_start, dt):
        pass

    return steps, update_step


def main():
    svl_tests.test_setup(TEST_NAME).set_duration(1200).set_freq(20).run_test(test)


if __name__ == "__main__":
    main()
