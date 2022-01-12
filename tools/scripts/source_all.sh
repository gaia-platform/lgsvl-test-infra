#!/bin/bash

ROS_FOXY=/opt/ros/foxy/setup.bash
if [ -f "${ROS_FOXY}" ]; then
    source ${ROS_FOXY}
fi

AUTOWARE=/opt/AutowareAuto/setup.bash;
if [ -f "${AUTOWARE}" ]; then
    source ${AUTOWARE};
fi;

ART_CORE=/opt/art_core/setup.bash;
if [ -f "${ART_CORE}" ]; then
    source ${ART_CORE};
fi;

ART_CORE_LOCAL=install/setup.bash;
if [ -f "${ART_CORE_LOCAL}" ]; then
    source ${ART_CORE_LOCAL};
fi;