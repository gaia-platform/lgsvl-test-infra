.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

# Default ADE_AUTOWARE_ARTIFACTS_TAG is defined by `autoware_auto_tag` at repo root.
set_ade_artifacts_tag = $(if $(ADE_AUTOWARE_ARTIFACTS_TAG $1),,$(eval ADE_AUTOWARE_ARTIFACTS_TAG := $(shell cat .gitlab-ci-autoware-tag.yml | sed -n "s/^.*ADE_AUTOWARE_ARTIFACTS_TAG:\s*\(\S*\).*$$/\1/p")))

# Default PRIVATE_TOKEN is defined by `install_private_token` inside of `tools/scripts` directory.
set_private_token = $(if $(PRIVATE_TOKEN $1),,$(eval PRIVATE_TOKEN := $(shell cat ./tools/scripts/install_private_token)))

.PHONY: clean
clean:
	@rm -rf build/ install/ log/ logs/

.PHONY: build
build:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --continue-on-error --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	else
		colcon build --continue-on-error --packages-up-to ${PACKAGES} --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	fi

.PHONY: test
test:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon test --packages-up-to $(shell cat packages_for_test.txt); colcon test-result --verbose
	else
		colcon test --packages-up-to ${PACKAGES}; colcon test-result --verbose
	fi

.PHONY: test-select
test-select:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon test --packages-select $(shell cat packages_for_test.txt); colcon test-result --verbose
	else
		colcon test --packages-select ${PACKAGES}; colcon test-result --verbose
	fi

.PHONY: reformat
reformat:
	source ./tools/scripts/source_all.sh
	autoflake --in-place --remove-unused-variables --remove-all-unused-imports --ignore-init-module-imports -r ${PATHS}
	black -l 99 ${PATHS}
	ament_uncrustify --reformat ${PATHS}

.PHONY: docker-host
docker-host:
	@(docker start docker_host || docker run --rm --name docker_host --cap-add=NET_ADMIN --cap-add=NET_RAW -d qoomon/docker-host) &>/dev/null

.PHONY: ade-update
ade-update: docker-host
	ade start --update

.PHONY: ade-update-gpu
ade-update-gpu: docker-host
	ade --rc .aderc-gpu start --update

.PHONY: ade-update-lgsvl
ade-update-lgsvl: docker-host
	ade --rc .aderc-lgsvl start --update

.PHONY: ade
ade:
	@ROOT_DIR=$${PWD##*/}
	ade enter " \
		cd $${ROOT_DIR}; \
		exec /bin/bash; \
	"

# This is used to download the `artifacts.zip` from a fork of AutowareAuto using the Gitlab CI script in the `tools/examples/gitlab_ci` directory.
.PHONY: download-autoware-artifacts
download-autoware-artifacts:
	@$(call set_ade_artifacts_tag)
	$(call set_private_token)
	curl -L --output tools/examples/ade_image/artifacts.zip --header "PRIVATE-TOKEN: ${PRIVATE_TOKEN}" "https://gitlab.com/api/v4/projects/${ADE_AUTOWARE_PROJECT_ID}/jobs/artifacts/${ADE_AUTOWARE_ARTIFACTS_TAG}/download?job=build_amd64_foxy"

# This build step assumes that the `artifacts.zip` and `ros-deps` are in the `tools/examples/ade_image` directory.
.PHONY: docker-build-ade-image
docker-build-ade-image:
	$(call set_ade_artifacts_tag)
	cd tools/examples/ade_image;
	docker build \
		--build-arg ROS_DISTRO=foxy \
		--build-arg CODENAME=focal \
		--build-arg ADE_IMAGE=registry.gitlab.com/${ADE_AUTOWARE_DOCKER_REPO}/amd64/ade-foxy:${ADE_AUTOWARE_ARTIFACTS_TAG} \
		--label ade_image_commit_sha="lgsvl-test-infra-ade-image-sha" \
		--label ade_image_commit_tag="lgsvl-test-infra-ade-image-tag" \
		-t lgsvl-test-infra-ade-image .

.PHONY: rosdep-install
rosdep-install:
	source ./tools/scripts/source_all.sh
	sudo apt-get update
	rosdep update
	rosdep install -y --ignore-src --from-paths .

# This is used to create the `ros-deps` list needed by the `ade_image` build.
.PHONY: rosdep-install-list
rosdep-install-list:
	source ./tools/scripts/source_all.sh
	rosdep update
	rosdep install --as-root "apt:false pip:false" --simulate --reinstall --ignore-src -y --from-paths . | sort >> tools/examples/ade_image/ros-deps
