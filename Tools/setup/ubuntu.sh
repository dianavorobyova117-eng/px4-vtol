#! /usr/bin/env bash

set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (22.04, 20.04, 18.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for nuttx, jMAVSim, Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - jMAVSim and Gazebo9 simulator (omit with arg: --no-sim-tools)
##

INSTALL_NUTTX="false"
INSTALL_SIM="true"
INSTALL_ARCH=`uname -m`

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi
done

# detect if running in docker
if [ -f /.dockerenv ]; then
	echo "Running within docker, installing initial dependencies";
	apt-get --quiet -y update && DEBIAN_FRONTEND=noninteractive apt-get --quiet -y install \
		ca-certificates \
		gnupg \
		lsb-core \
		sudo \
		wget \
		;
fi

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# # check requirements.txt exists (script not run in source tree)
# REQUIREMENTS_FILE="requirements.txt"
# if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
# 	echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as ubuntu.sh (${DIR})."
# 	return 1
# fi


# check ubuntu version
# otherwise warn and point to docker?
UBUNTU_RELEASE="22.04"

echo "Installing PX4 general dependencies"

sudo apt-get update -y --quiet
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
	astyle \
	build-essential \
	cmake \
	cppcheck \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	libfuse2 \
	libxml2-dev \
	libxml2-utils \
	make \
	ninja-build \
	python3 \
	python3-dev \
	python3-pip \
	python3-setuptools \
	python3-wheel \
	rsync \
	shellcheck \
	unzip \
	zip \
	;

# Python3 dependencies
echo
echo "Installing PX4 Python3 dependencies"
if [ -n "$VIRTUAL_ENV" ]; then
	# virtual environments don't allow --user option
	python -m pip install -r ${DIR}/requirements.txt
else
	# older versions of Ubuntu require --user option
	python3 -m pip install --user -r ${DIR}/requirements.txt
fi


if [[ $INSTALL_SIM == "true" ]]; then

	echo "[ubuntu.sh] Installing PX4 simulation dependencies"

	# General simulation dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		bc \
		;

	# Gazebo
	# Expects Ubuntu 22.04 > by default
	echo "[ubuntu.sh] Gazebo (Harmonic) will be installed"
	echo "[ubuntu.sh] Earlier versions will be removed"
	# Add Gazebo binary repository
	sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	sudo apt-get update -y --quiet

	# Install Gazebo
	gazebo_packages="gz-harmonic libunwind-dev"


	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		dmidecode \
		$gazebo_packages \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		libeigen3-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler \
		;

	if sudo dmidecode -t system | grep -q "Manufacturer: VMware, Inc." ; then
		# fix VMWare 3D graphics acceleration for gazebo
		echo "export SVGA_VGPU10=0" >> ~/.profile
	fi

fi