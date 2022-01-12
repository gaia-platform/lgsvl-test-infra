ARG ADE_IMAGE
ARG ROS_DISTRO=dashing
ARG CODENAME=bionic

FROM $ADE_IMAGE

COPY artifacts.zip /tmp/
RUN unzip /tmp/artifacts.zip -d /tmp/ && \
    tar xzf /tmp/opt.tar.gz -C / && \
    rm /tmp/artifacts.zip && \
    rm /tmp/opt.tar.gz

# Ensure we have the latest certificates installed before calling apt-get
RUN apt-get update ; apt-get install ca-certificates

# Add apt-add-repository command.
RUN apt-get update && \
    apt-get install -y \
        software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Add AutonomouStuff Kvaser PPA
RUN apt-add-repository -y ppa:astuff/kvaser-linux && \
    rm -rf /var/lib/apt/lists/*

# Install from ros-deps file.
COPY ros-deps /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /opt/AutowareAuto/setup.sh && \
    apt-get update && \
    /bin/bash /tmp/ros-deps && \
    rm -rf /var/lib/apt/lists/* /tmp/ros-deps

# Install from apt-packages file.
COPY apt-packages /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /opt/AutowareAuto/setup.sh && \
    apt-get update && \
    apt-get install -y ros-$ROS_DISTRO-lgsvl-bridge clang-tidy && \
    apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-packages

# Install from pip3-packages file.
COPY pip3-packages /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /opt/AutowareAuto/setup.sh && \
    pip3 install -U pip-tools black autoflake flake8-eradicate eradicate transforms3d pymap3d && \
    # NOTE(dvd): Force a downgrade on pycodestyle because 2.7 is incompatible with ament-flake8.
    pip3 install -Iv pycodestyle==2.6.0 && \
    pip3 install -U \
        $(cut -d# -f1 </tmp/pip3-packages) && \
    rm -rf /root/.cache /tmp/pip-* /tmp/pip3-packages

# Install Allied Vision Vimba SDK
RUN wget https://autoware-auto.s3.us-east-2.amazonaws.com/iac/Vimba_v4.2_Linux.tgz && \
    tar xvf Vimba_v4.2_Linux.tgz && \
    rm Vimba_v4.2_Linux.tgz && \
    mv Vimba_4_2/ /opt && \
    /opt/Vimba_4_2/VimbaGigETL/Install.sh && \
    cp /opt/Vimba_4_2/VimbaCPP/DynamicLib/x86_64bit/libVimbaCPP.so /usr/lib && \
    cp /opt/Vimba_4_2/VimbaCPP/DynamicLib/x86_64bit/libVimbaC.so /usr/lib

# Install lgsvl Python API.
RUN mkdir -p /opt/lgsvl/PythonAPI && \
    git clone https://github.com/lgsvl/PythonAPI.git /opt/lgsvl/PythonAPI && \
    cd /opt/lgsvl/PythonAPI && \
    pip3 install -r requirements.txt . && \
    rm -rf /root/.cache /tmp/pip-* /tmp/pip3-packages && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-packages

COPY entrypoint /lgsvl_test_infra_entrypoint
ENTRYPOINT ["/lgsvl_test_infra_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
