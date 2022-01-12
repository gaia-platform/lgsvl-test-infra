# Copyright (c) 2021-2022 Gaia Platform LLC
# SPDX-License-Identifier: MIT

from __future__ import print_function
from environs import Env
from ament_index_python.packages import get_package_share_directory
import sys
import os
import lgsvl
import threading
import time
import subprocess
import signal
import csv
import rclpy
import random
from lgsvl.geometry import Spawn, Transform, Vector


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def force_exit(err):
    try:
        sys.exit(err)
    except SystemExit:
        os._exit(err)


def process_exists(name):
    ps = subprocess.Popen("ps -A", shell=True, stdout=subprocess.PIPE)
    output = ps.stdout.read().decode("utf-8")
    ps.stdout.close()
    ps.wait()
    for line in output.split("\n"):
        if line != "" and line is not None:
            fields = line.split()
            pname = fields[3]
            if pname == name:
                return True
    return False


class Region:
    def __init__(self, tl, tr, bl, br):
        self.tl = tl
        self.tr = tr
        self.bl = bl
        self.br = br

    def contains(self, point):
        if (
            point.x > self.tl.x
            and point.x < self.bl.x
            and point.z > self.tl.z
            and point.z < self.tr.z
        ):
            return True
        return False

    def visualize(self, sim):
        rotation = Vector(0.0, 0.0, 0.0)
        sim.spawn_pylon(Spawn(Transform(self.tl, rotation)))
        sim.spawn_pylon(Spawn(Transform(self.tr, rotation)))
        sim.spawn_pylon(Spawn(Transform(self.bl, rotation)))
        sim.spawn_pylon(Spawn(Transform(self.br, rotation)))


class TestState:
    def __init__(self, test_env):
        self.in_start_finish = False
        self.in_back_straight = False
        self.in_turn_four_exit = False
        self.num_laps = 0
        self.last_step_time = 0
        self.time_since_start = 0
        self.current_step = 0
        self.num_steps = 0
        self.test_env = test_env

    def update(self, time_since_start):
        self.time_since_start = time_since_start
        if self.current_step < self.num_steps:
            if self.steps[self.current_step].is_condition_met(self):
                self.steps[self.current_step].on_trigger(self.test_env)
                self.go_to_next_step()

    def set_steps(self, steps):
        self.steps = steps
        self.num_steps = len(self.steps)

    def go_to_next_step(self):
        self.last_step_time = self.time_since_start
        self.current_step += 1

    def get_current_step(self):
        return self.current_step

    def time_since_last_step(self):
        return self.time_since_start - self.last_step_time


class TestStep:
    def __init__(self, wait_for=None, trigger=None, action=None):
        self.wait_for = wait_for
        self.trigger = trigger
        self.action = action

    def is_condition_met(self, test_state):
        if self.wait_for is not None and self.trigger is not None:
            wait_for_passed = test_state.time_since_last_step() >= self.wait_for
            return wait_for_passed and self.trigger(test_state)
        elif self.wait_for is not None:
            wait_for_passed = test_state.time_since_last_step() >= self.wait_for
            return wait_for_passed
        elif self.trigger is not None:
            return self.trigger(test_state)
        else:
            return True

    def on_trigger(self, test_env):
        self.action(test_env)


def set_flag(flag):
    def inner(_):
        flag_type = "track_flag"
        if flag == "black":
            flag_type = "vehicle_flag"
        args = ["ros2", "param", "set", "lgsvl_mocks", flag_type, flag]
        print(f"Setting {flag_type} {flag}")
        subprocess.run(args, capture_output=True)

    return inner


def turn_switch(switch):
    def inner(_):
        args = ["ros2", "param", "set", "lgsvl_mocks", "switch_position", str(switch)]
        print(f"Turning to switch {switch}")
        subprocess.run(args, capture_output=True)

    return inner


def setup_pylons(test_env):
    distance = random.randrange(20.0, 100.0)

    # Gate 1
    test_env.spawn_pylon(Spawn(Transform(Vector(distance, 2.0, 0.5))))
    test_env.spawn_pylon(Spawn(Transform(Vector(distance, 2.0, 3.5))))

    # Gate 2
    test_env.spawn_pylon(Spawn(Transform(Vector(distance + 100.0, 2.0, -5.5))))
    test_env.spawn_pylon(Spawn(Transform(Vector(distance + 100.0, 2.0, -3.0))))


def default_start_steps():
    steps = [
        TestStep(wait_for=0.5, action=turn_switch(3)),
        TestStep(wait_for=0.5, action=turn_switch(4)),
        TestStep(wait_for=1.0, action=set_flag("orange")),
    ]

    return steps


def start_lgsvl_bridge():
    app = "lgsvl_bridge"
    if process_exists(app):
        print("Bridge already running")
        return

    print("Starting bridge")
    args = []
    lgsvl_bridge = subprocess.Popen([app] + args, preexec_fn=os.setsid)
    time.sleep(1)
    return lgsvl_bridge


def start_ego():
    print("Starting ego")
    app = "ros2"
    args = ["launch", "svl_tests", "svl.launch.py", "use_sim_time:=True"]
    ego = subprocess.Popen([app] + args, preexec_fn=os.setsid)
    time.sleep(1)
    return ego


def connect_to_sim(lgsvl_host_name, lgsvl_port):
    def excepthook(args):
        eprint(args)
        force_exit(1)

    try:
        # Set excepthook to catch connection failure.
        threading.excepthook = excepthook
        print(f"Connecting to simulation at {lgsvl_host_name}:{lgsvl_port}")
        sim = lgsvl.Simulator(lgsvl_host_name, lgsvl_port)
        print(f"Connected to simulation at {lgsvl_host_name}:{lgsvl_port}")
        return sim
    except Exception:
        force_exit(1)


def stop_process_group(p, mode="SIGINT"):
    stop_timeout_s = 2.0

    # Get the process group ID.
    process_group_id = os.getpgid(p.pid)
    return_val = None
    if mode == "SIGINT" or return_val is None:
        try:
            os.killpg(process_group_id, signal.SIGINT)
            return_val = p.wait(stop_timeout_s)
        except Exception:
            eprint("Process did not exit after SIGINT, sending SIGTERM.")
    if mode == "SIGTERM" or return_val is None:
        try:
            os.killpg(process_group_id, signal.SIGTERM)
            return_val = p.wait(stop_timeout_s)
        except Exception:
            eprint("Process did not exit after SIGTERM, sending SIGKILL.")
    if mode == "SIGKILL" or return_val is None:
        os.killpg(process_group_id, signal.SIGKILL)
        return_val = p.wait()
    return return_val


def read_env(env, env_path):
    print(f"Reading environment from {env_path}")
    env.read_env(env_path, recurse=False)


class SvlTestState:
    def __init__(self, sim_id, msg):
        env = Env()

        env_file_name = ".svl_tests_env"
        if os.path.exists(env_file_name):
            # Read from <CWD>/.svl_tests_env if it exists.
            read_env(env, env_file_name)
        else:
            eprint("Create file .svl_tests_env in current working directory.")
            eprint("See README.md for details.")
            force_exit(1)

        self.CLOUD_URL = env.str("SVL_TESTS__CLOUD_URL", "https://wise.svlsimulator.com")
        self.SCENARIO = env.str("SVL_TESTS__SCENARIO", "8e47b1b5-1f57-419e-a99b-cd53438ebe63")
        self.AUTOPILOT_HOST = env.str("SVL_TESTS__AUTOPILOT_HOST", "localhost")
        self.AUTOPILOT_PORT = env.int("SVL_TESTS__AUTOPILOT_PORT", 9090)
        self.SIM_HOST = env.str("SVL_TESTS__SIM_HOST", "docker_host")
        self.SIM_PORT = env.int("SVL_TESTS__SIM_PORT", 8181)
        self.SIM_ID = sim_id
        self.EGO_SENSOR_UUID = env.str("SVL_TESTS__EGO_SENSOR_UUID", "")
        self.BOGEY_SENSOR_UUID = env.str("SVL_TESTS__BOGEY_SENSOR_UUID", "")

        self.bridge = start_lgsvl_bridge()
        self.sim = connect_to_sim(self.SIM_HOST, self.SIM_PORT)
        self.duration = 30
        self.freq = 20
        self.bogies = []
        self.msg = msg

    def load_ims(self):
        ims_map_uuid = "62765742-57bf-4ccd-85e5-db8295d34ead"
        if self.sim.current_scene == ims_map_uuid:
            self.sim.reset()
        else:
            self.sim.load(ims_map_uuid)
        return self.sim.get_spawn()

    def spawn_ego_car(self, spawn, velocity=0):
        print(f"Placing ego {self.EGO_SENSOR_UUID} at {spawn} with velocity {velocity}")
        state = lgsvl.AgentState()
        state.transform = spawn
        state.velocity = velocity * lgsvl.utils.transform_to_forward(spawn)

        ego = self.sim.add_agent(self.EGO_SENSOR_UUID, lgsvl.AgentType.EGO, state)
        self.ego = ego

        self.connect_agent_to_bridge(ego, self.AUTOPILOT_HOST, self.AUTOPILOT_PORT)
        self.ros2 = start_ego()

        return ego

    def spawn_bogey_car(self, spawn, velocity=0):
        print(f"Placing bogey {self.BOGEY_SENSOR_UUID} at {spawn} with velocity {velocity}")
        state = lgsvl.AgentState()
        state.transform = spawn
        state.velocity = velocity * lgsvl.utils.transform_to_forward(spawn)

        bogey = self.sim.add_agent(self.BOGEY_SENSOR_UUID, lgsvl.AgentType.EGO, state)
        self.bogies.append(bogey)
        return bogey

    def spawn_pylon(self, spawn):
        state = lgsvl.ObjectState()
        state.transform = spawn
        return self.sim.controllable_add("LGSVLPylon", state)

    def connect_agent_to_bridge(self, agent, bridge_host, bridge_port):
        agent.connect_bridge(bridge_host, bridge_port)
        print("Waiting for connection...")
        while not agent.bridge_connected:
            time.sleep(1)
        return agent.bridge_connected

    def try_stop_ego(self, err):
        if hasattr(self, "ros2"):
            try:
                print("Stopping ego")
                ret = stop_process_group(self.ros2)
                if ret != 0:
                    err = ret
            except Exception:
                err = 1
        return err

    def try_stop_bridge(self, err):
        if hasattr(self, "bridge"):
            try:
                print("Stopping bridge")
                ret = stop_process_group(self.bridge)
                if ret != 0:
                    err = ret
            except Exception:
                err = 1
        return err

    def try_stop_client(self, err):
        err = self.try_stop_ego(err)
        err = self.try_stop_bridge(err)
        return err

    def end_simulation(self, err):
        err = self.try_stop_client(err)

        try:
            print("Stopping simulation")
            self.sim.stop()
            print("Resetting simulation")
            self.sim.reset()
            print("Closing simulation")
            self.sim.close()
        except Exception:
            err = 1

        force_exit(err)

    def set_duration(self, duration):
        self.duration = duration
        return self

    def set_freq(self, freq):
        self.freq = freq
        return self

    def set_logging(self, logging_enabled):
        self.logging_enabled = logging_enabled
        return self

    def run_test(self, init_func):
        return_code = 0
        try:
            # Create mocks.

            rclpy.init()

            # Setup shared triggers & test state.

            state = TestState(self)

            # TODO(user): This is hard-coded for IMS in SVL 2021.2 -- you probably will want to remove these regions!

            start_finish = Region(
                Vector(-5.0, 2.0, -20.0),
                Vector(-5.0, 2.0, 10.0),
                Vector(5.0, 2.0, -20.0),
                Vector(5.0, 2.0, 10.0),
            )

            back_straight = Region(
                Vector(180.0, 2.0, 700.0),
                Vector(180.0, 2.0, 740.0),
                Vector(200.0, 2.0, 700.0),
                Vector(200.0, 2.0, 740.0),
            )

            turn_four_exit = Region(
                Vector(-690.0, 2.0, -20.0),
                Vector(-690.0, 2.0, 10.0),
                Vector(-680.0, 2.0, -20.0),
                Vector(-680.0, 2.0, 10.0),
            )

            # Run the test-specific init function.

            steps, update_step = init_func(self)
            state.set_steps(steps)

            # Time-stepped sim loop.

            step_time = 1.0 / self.freq
            steps = int(self.duration * self.freq)
            t0 = self.sim.current_time
            for _ in range(steps):
                t1 = self.sim.current_time

                # Run the sim.

                self.sim.run(time_limit=step_time)

                # Spin the mocks.

                t2 = self.sim.current_time

                # Find trigger conditions.

                state.crossed_start_finish = False
                state.crossed_back_straight = False
                state.crossed_turn_four_exit = False

                ego_state = self.ego.state

                entered_start_finish = start_finish.contains(ego_state.transform.position)
                if not state.in_start_finish and entered_start_finish:
                    state.in_start_finish = True
                if state.in_start_finish and not entered_start_finish:
                    state.in_start_finish = False
                    state.crossed_start_finish = True

                entered_back_straight = back_straight.contains(ego_state.transform.position)
                if not state.in_back_straight and entered_back_straight:
                    state.in_back_straight = True
                if state.in_back_straight and not entered_back_straight:
                    state.in_back_straight = False
                    state.crossed_back_straight = True

                entered_turn_four_exit = turn_four_exit.contains(ego_state.transform.position)
                if not state.in_turn_four_exit and entered_turn_four_exit:
                    state.in_turn_four_exit = True
                if state.in_turn_four_exit and not entered_turn_four_exit:
                    state.in_turn_four_exit = False
                    state.crossed_turn_four_exit = True

                if state.crossed_start_finish:
                    state.num_laps += 1

                # Update the test state.

                state.update(t2 - t0)

                # Run the test-specific update function.

                update_step(state, t2 - t0, t2 - t1)

        except KeyboardInterrupt:
            print("Interrupting simulation")
            return_code = self.try_stop_client(1)
        except Exception as ex:
            eprint(ex)
            return_code = self.try_stop_client(1)

        try:
            rclpy.shutdown()
            self.end_simulation(return_code)
        except KeyboardInterrupt:
            self.end_simulation(1)
        except Exception as ex:
            eprint(ex)
            self.end_simulation(1)


def test_setup(msg):
    SIM_ID = ""
    return SvlTestState(SIM_ID, msg)
