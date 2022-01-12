# Background

This set of plugins & infrastructure provides mocks for various sensors and drivers on the Dallara IL-15 during the Indy Autonomous Challenge at IMS in 2021.
This code is provided by Gaia Platform LLC in collaboration with the University of Hawai'i and University of San Diego AI Racing Tech Team.

# Assumptions

## `IACBaseSoftware`

`IACBaseSoftware` was not used.

- The plugins were developed against a patch set maintained on top of the main `AutowareAuto` -- **not** the IACBaseSoftware fork.
- `raptor-dbw-ros2` was forked from https://github.com/NewEagleRaptor/raptor-dbw-ros2 to preserve compatibility with `AutowareAuto`'s already provided `ne_raptor_interface`, instead of using the IACBaseSoftware fork. Our implementation can be found here: https://github.com/gaia-platform/raptor-dbw-ros2.
- The state machine in `dbw_state_machine` on the IACBaseSoftware fork of AutowareAuto was implemented in a custom component so that it could be run in SVL on top of the `lgsvl_interface` and `LGSVLRaptorSensor`.
- Custom ROS2 messages were defined in `iac_common` and `iac_msgs` instead of using the `deep_orange_msgs`.

## `AutowareAuto`

It's assumed that the software deployed to the vehicle used the `ne_raptor_interface` in `AutowareAuto` and that in simulation under `SVL` that interface was replaced by the `lgsvl_interface`. This repo contains the patch set needed to allow the `RawControlCommand` and manual gear commands in those components.

## Test Like You Drive

This was used for full stack testing of our vehicle software -- from pit, to warmup, through the racing laps, and back to pit, including the startup sequence with Raptor and management of Race Control flags.

Our goal was to test as much software as possible in SVL, so that the _only_ things we replaced were the lowest level vehicle controllers (`ne_raptor_interface`, `raptor-dbw-ros2`) and sensors. Everything else we ran in SVL and on the vehicle identically.

| Software on Vehicle | Software in Sim | ROS2 Interface |
|---------------------|-----------------|-----------|
| ne_raptor_interface | lgsvl_interface | autoware_auto_msgs |
| raptor-dbw-ros2     | lgsvl_mocks, LGSVLRaptorSensor, LGSVLVppWheelSpeedSensor | raptor_dbw_msgs, iac_msgs |
| Sensor drivers      | lgsvl_mocks, SVL sensor plugins | The sensor messages used by the real driver |
| Everything else     | The same | ... |

## `lgsvl_mocks`

SVL messages do not match the real messages used on the car, especially for SVL provided sensors, so we used `lgsvl_mocks` to wrap & republish messages using `ROS2` definitions from `novatel_oem7_msgs`, `iac_msgs`, `raptor_dbw_msgs`, `nav_msgs`, and `sensor_msgs`.

This is also where we inserted code to transform SVL coordinates into "real" coordinates, so that the vehicle software did not require any configuration prior to driving in the sim. E.g. We fixed the offset / rotation of the IMS map here.

## `iac_msgs`

This repo contains portions of C++ ROS2 projects to replace IAC messages defined at https://gitlab.com/IACBaseSoftware/deep_orange_msgs/-/tree/master/msg. The messages are used by the `LGSVLRaptorSensor` plugin and `lgsvl_mocks` to mock the startup between Raptor and the vehicle software in SVL.

## `iac_common`

This repo contains portions of C++ ROS2 messages to work with the `iac_msgs` and simplify `pub-sub` code in ROS2. In general, we preferred to use `SensorDataQoS` and send messages at a known frequency for control code.

## ADE

We used an ADE image (https://ade-cli.readthedocs.io/en/latest/intro.html) for our development. Our ADE image included our custom build of AutowareAuto, the SVL Python API (https://github.com/lgsvl/PythonAPI), our pre-built vehicle software, and all of our dependencies.

| Layer | ADE Image | Contained |
|--------|-----|-------------|
| 0 | ROS2 | ROS2 Foxy |
| 1 | AutowareAuto | Updated `ne_raptor_interface`, `lgsvl_interface` |
| 2 | Our mono-repo | `lgsvl_mocks`, `svl_mocks`, `iac_msgs`, `iac_common`, `PythonAPI`, vehicle software & dependencies |

## SVL

These plugins and patch sets assume custom builds of SVL and the sensor plugins.

The custom build of SVL is needed for `Vehicle Physics Pro` (used by the updated `DallaraIL15` vehicle plugin) and for fixes to SVL vehicle dynamics.
SVL did not allow overlapped throttle and braking, and it also did not support manual gear changes, so both of those issues were fixed in these changes.
The `ROS2` bridge and `LGSVLControlSensor` also required updates, for the same reasons.

## `svl_tests`

This is the actual entry point for running tests. This will launch start the `lgsvl_bridge` if it isn't running already, connect to the SVL `PythonAPI`, handle the spawning of the agent & launching of any associated ROS2 code, and teardown all of those processes when those test is finished or cancelled. The tests are written in Python and can be run either in real time or time-stepped with the `set_freq` argument.

The software that we ran during tests included the same telemetry that we would use for real on-track runs. We used that telemetry to pass or fail the SVL tests and to anchor our simulation results to our on-track data.

## Simulated Bogeys

Vehicle Physics Pro's free version does not support more than one car in the scene. If you want to use Vehicle Physics Pro on multiple cars, you can replace the Vehicle Physics Pro asset in `simulator\Assets\Vehicle Physics Pro` with the Pro or Enterprise edition.

The way we ran this for the 2021 IAC at IMS was to rely on time stepping and simulate the bogey, e.g. using a non-VPP Dallara IL-15 like an SVL NPC or just the "default" Dallara IL-15 with a very simple controller on it. Our rationale was that we wanted specific, deterministic paths for the AI to follow so that we would have repeatable test cases from build to build as we iterated on and improved our own algorithms. It's also more performant since the AI cars can just be simple lane followers without full stack simulations or even sensor data.

# Building the Custom Plugins

## Building AutowareAuto

Clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto and checkout commit `5b95c59de1f3e616be4886b9d65df46242495b26`.

Apply the patchset in `src/external/AutowareAuto`.

Follow https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/building.html to build.

We maintained a fork of AutowareAuto with this patch set. It contains minimal changes from the upstream repo. Our repo had CI that built, tested, and packaged `AutowareAuto` into an `ADE` volume.

Our `.gitlab-ci` script is included in `tools/examples/gitlab-ci/AutowareAuto`.

## Building a Mono-Repo

This repo is a shell of the mono-repo we used for the competition. We preferred a mono-repo to minimize the amount of dependencies that we needed to tag. This is where our drivers, launch files, documentation, tooling, and other custom software lived. This repo had CI that built, tested, and packaged a _specific tag_ of `AutowareAuto` into an `ADE` container with binaries of our other software and any specific dependencies, like `Gaia`.

One benefit of separating our code from the `AutowareAuto` codebase (unlike the `IACBaseSoftware` fork) is that we could iterate more quickly on changes -- we very rarely had to change `AutowareAuto` and had faster CI times. The `.gitlab-ci` script we used is in `tools/examples/gitlab-ci/.gitlab-ci-autoware.yml`.

Our `.gitlab-ci` script is included in `tools/examples/gitlab-ci/.gitlab-ci-lgsvl-test-infra.yml`. The file `.gitlab-ci-autoware-tag.yml` was in the same directory.

## Building SVL & Plugins

Clone https://github.com/gaia-platform/simulator/ and checkout branch `gaia/release-2021.3/iac`.
Then clone the other repos and replace them in the following directories:

- simulator/Assets/External/Controllables/LGSVLPylon https://github.com/gaia-platform/LGSVLPylon
- simulator/Assets/External/Bridges/ROS2 https://github.com/gaia-platform/ROS2 (branch: `gaia/2021.3/iac`)
- simulator/Assets/External/Sensors/LGSVLControlSensor https://github.com/gaia-platform/LGSVLControlSensor (branch: `gaia/2021.3/iac`)
- simulator/Assets/External/Sensors/LGSVLRaptorSensor https://github.com/gaia-platform/LGSVLRaptorSensor
- simulator/Assets/External/Sensors/LGSVLVppWheelSpeedSensor https://github.com/gaia-platform/LGSVLVppWheelSpeedSensor/
- simulator/Assets/External/Vehicles/DallaraIL15 https://github.com/lgsvl/DallaraIL15 (branch: `gaia/2021.3/iac`)

Build the simulator using the guide at https://www.svlsimulator.com/docs/installation-guide/build-instructions/.

Build the plugins using the guide at https://www.svlsimulator.com/docs/plugins/sensor-plugins/.

Upload the plugins to your SVL account at https://wise.svlsimulator.com/.
**Set the visibility of the plugins to private during the upload.**

Create a sensor configuration for the Dallara IL-15. **Use the custom ROS2 bridge and LGSVLControlSensor.**

The sensor config should have the following sensors at minimum:
- CAN Bus Sensor (**Topic**: `/state_report`, **Frequency**: 100)
- Clock Sensor (**Topic**: `/clock`, )
- Transform Sensor (`rear_axis_middle`)
    - Transform Sensor (`rear_axis_middle_ground`)
        - GPS Device Sensor (**Topic**: `/lgsvl/gnss_fix`, **IgnoreMapOrigin**: No, **Frequency**: 20)
        - GPS Odometry Sensor (**Topic**: `/lgsvl/gnss_odom`, **IgnoreMapOrigin**: No, **Frequency**: 20, **ChildFrame**: `base_link`)
        - Vehicle Odometry Sensor (**Topic**: `/vehicle_odom`, **Frequency**: 20)
        - IMU Sensor (**Topic**: `/lgsvl/imu`)
- LGSVLVppWheelSpeedSensor (**Topic**: `/lgsvl/wheel_speeds`)
- LGSVLRaptorSensor (**Topic**: `/raptor_dbw_ros2/ct_report`)
- LGSVLControlSensor (**Topic**: `/vehicle_control_cmd`)

# Known Issues

- The `ROS2` bridge plugin and the `LGSVLControlSensor` plugin might require custom names to not conflict with SVL default plugins.
- The `svl_tests` assume that the map is IMS and hard-code certain regions inside of `svl_tests.py` for tracking # of laps completed, e.g. to spawn pylons or trigger race control flags.
- You'll need to provide your own launch file in `test/svl_tests/launch/svl.launch.py`. This and other required TODOs are marked by `TODO(user)`.
