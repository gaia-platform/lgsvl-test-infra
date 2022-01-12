# LGSVL Test Infrastructure

## Acknowledgments

This code is provided by Gaia Platform LLC in collaboration with the University of Hawai'i and University of San Diego AI Racing Tech Team.

## Cloning

```bash
cd ~
mkdir -p adehome
cd adehome

# Clone using...
git clone https://github.com/gaia-platform/lgsvl-test-infra.git lgsvl_test_infra # HTTPS
# ...or..
git clone git@github.com:gaia-platform/lgsvl-test-infra.git lgsvl_test_infra # SSH
```

## Organization

This repository is organized into the following directories.

- `docs`
- `src`
- `test`
- `tools`

### `docs`

Contains documentation on how to build & use this code. This is _not_ batteries included! This will require work to integrate into your own project.

### `src`

Contains source code intended for deployment to the AV-21.
The `external/AutowareAuto` directory contains a patch set for `AutowareAuto`.

### `test`

Contains source code, data, or tooling for tests.

### `tools`

Contains tools for use outside of the vehicle like the `ade_image` or various build `scripts`.

## ADE

This repo contains example `.aderc` files.

It's assumed that you'll provide your own Docker layers for an ADE container including `ROS`, the `LGSVL` Python API, `lgsvl_mocks`, `svl_tests`, and the latest built binaries of `AutowareAuto`, your vehicle controller, and any other dependencies.

### Installing ADE

1. Install any dependencies needed by https://ade-cli.readthedocs.io/en/latest/install.html.
2. Make sure you follow the Docker Linux Post-Install Instructions after installing Docker https://docs.docker.com/engine/install/linux-postinstall/.
3. Restart computer.
4. Finish installing ADE as listed on https://ade-cli.readthedocs.io/en/latest/install.html.

### Examples

The `tools/examples` folder contains a `Dockerfile` for building an ADE image and examples of our `.gitlab-ci` scripts.

##  Makefile

The `Makefile` defines several `.phony` targets for convenience like `make clean` or `make build`.

The `Makefile` provides commands for stepping into the ADE container.
These commands use `docker-host` from https://github.com/qoomon/docker-host to forward network traffic between the container and your host computer. In our testing, we found that this worked significantly better than `--net=host` and supported every combination of `ROS2` code location (e.g. in a VM, running natively on Ubuntu, inside of WSL) and SVL location (on the same PC, in a VM, across the network).

```bash
# This pulls & starts the most up to date container.
make ade-update

# Enter the container.
make ade
```

The expected directory layout is something like the following:

```bash
~\$USER\adehome\
    \<your repo>\
    \AutowareAuto\ # (optional)
    ..
```

#### **Useful Commands**

```bash
# Source everything, preferring local build.
# You can put this in your .bashrc.
source ./tools/scripts/source_all.sh
```

```bash
# Create a local container using ADE_AUTOWARE_ARTIFACTS_TAG as base.
# Assumes that `ros-deps` and `artifacts.zip` are located in `tools\examples\ade_image` directory.
# If no ADE_AUTOWARE_ARTIFACTS_TAG is provided, it will use the `.gitlab-ci-autoware-tag.yml` file.
make docker-build-ade-image ADE_AUTOWARE_ARTIFACTS_TAG=autoware-lgsvl-test-infra-1.0.0
```

```bash
# Build a specific set of packages.
make build PACKAGES='svl_tests iac_common'
```

```bash
# Run tests on packages. CI runs on all packages in the `packages_for_test.txt` file at
# the base of the repo, but this command will allow you run them locally for select packages.
# If no PACKAGES argument is provided, this will use the `packages_for_test.txt` file.
make test PACKAGES='svl_tests iac_common'
```

```bash
# Run tests on packages, ignoring their dependencies.
# If no PACKAGES argument is provided, this will use the `packages_for_test.txt` file.
make test-select PACKAGES='svl_tests iac_common'
```

```bash
# Run ament-uncrustify on paths. This will reformat any C++ not matching the format guidelines.
# This also runs Black and autoflake for formatting Python code on the paths.
make reformat PATHS='src/common/iac_common/'
```

```bash
# Downloads artifacts.zip from ADE_AUTOWARE_ARTIFACTS_TAG to `tools\examples\ade_image` directory.
# If no ADE_AUTOWARE_ARTIFACTS_TAG is provided, it will use the `.gitlab-ci-autoware-tag.yml` file.
# If no PRIVATE_TOKEN is provided, it will use the `install_deploy_token` file from tools\scripts.
make download-autoware-artifacts ADE_AUTOWARE_ARTIFACTS_TAG=autoware-lgsvl-test-infra-1.0.0 PRIVATE_TOKEN=<private>
```

```bash
# Installs any missing ROS dependencies. Use this when you're developing locally and you've added
# a new package to the repo. Any containers built by CI already have all necessary ROS dependencies
# installed in them.
make rosdep-install
```

```bash
# Creates `ros-deps` file in `tools\examples\ade_image` directory. This is a list of packages that would
# be installed by running `make rosdep install`.
make rosdep-install-list
```

## `svl_tests`

### Prerequisites

1. Build SVL following the instructions in `docs`.
2. **IMPORTANT:** Open `config.yml` in SVL directory and edit line `api_hostname: "localhost"` to `api_hostname: "*"`, then save. If you don't do this, you'll likely fail to connect to the SVL instance across a network!
3. Create an account at https://wise.svlsimulator.com/ if needed.
3. Open SVL and pair with with online account. Create a cluster if needed.
4. Create a new scenario on cluster from step 3 with type `API Only`.
5. If running SVL on a separate computer, you may need to expose ports `8181` and `9090` in the firewall settings on both machines.

### Configuring environment

Create an `.svl_tests_env` file in the base of the repo, e.g. `~/adehome/lgsvl_test_infra`.

**Example contents:**

```bash
# The external IP of the machine running the lgsvl_bridge and ROS2 nodes.
# If you're running the lgsvl_bridge and ROS2 nodes inside of the ADE container,
# then this is the IP of the computer running the container -- if the container
# is inside of WSL2, then it's the IP of the Windows machine.
# Leave this commented out if you're running SVL on the same machine.

#SVL_TESTS__AUTOPILOT_HOST="1.2.3.4"

# The external IP of the machine running the SVL simulator.
# Leave this commented out if you're running SVL on the same machine outside of ADE.
# If you're running SVL on the same machine inside of ADE, set this to "localhost".

#SVL_TESTS__SIM_HOST="1.2.3.6"

# The GUID for the ego sensor config of the Dallara IL-15.

# IMPORTANT: When creating the configuration, choose the custom `ROS2 Bridge`.
# This must be done manually for your SVL account through the web browser GUI.

SVL_TESTS__EGO_SENSOR_UUID="00000000-0000-0000-0000-000000000000"

# The GUID for the bogey sensor config. This should be a different GUID from above.

# IMPORTANT: Creating the configuration, choose `No bridge`.
# This must be done manually for your SVL account through the web browser GUI.

SVL_TESTS__BOGEY_SENSOR_UUID="00000001-0001-0001-0001-000000000001"

# The GUID for an `API Only` scenario.

SVL_TESTS__SCENARIO="00000002-0002-0002-0002-000000000002"
```

### Running a test from `svl_tests`

1. Open SVL.
2. From web browser, start the scenario.
3. From inside of ADE container, enter `ros2 run svl_tests <test_name>`. Tab should auto-complete available tests, e.g. `ims_demo`.
4. You can use CTRL-C to interrupt the test & cleanup.
5. You can run as many tests as you want without needing to restart the scenario.
