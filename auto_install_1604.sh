#!/bin/bash

# ===============================================================================
# File:  auto_install_1604.sh
#
# Updated 6/19/2017 by Chase Murray
#
# NOTES:
#	* This script installs the software for ROS Kinetic on Ubuntu 16.04.
#		- We are using ROS Indigo on Ubuntu 14.04 in the lab. 
#
# INSTALLATION INSTRUCTIONS: 
#	1) Download this script to the location of your choice.  ~/Downloads is fine.
#	2) Change directories to the location where this script is saved.
#	3) chmod +x auto_install_1604.sh
#	4) ./auto_install_1604.sh
#
# *****************CAUTION***********************
#	* This script will OVERWRITE anything in the following directories:
#		- ~/catkin_ws
#		- ~/mavlink
#		- ~/dronekit-python
#		- ~/dronekit-sitl
#		- ~/MAVProxy
#		- ~/jMAVSim
#		- ~/cesium
#		- ~/cesium-3d-tiles
#		- ~/qgroundcontrol
#		- ~/pyaudio
#		- ~/blather
#		- ~/.config/blather/
#	* MAKE. SURE. YOU. HAVE. BACKUPS.
# ===============================================================================

# CHANGELOG:
# 	Replaced "indigo" with "kinetic"
#	Replaced "python-wxgtk2.8" with "python-wxgtk3.0"
#	Replaced "python-gst0.10" with "python-gst-1.0"
#	Replaced "gstreamer0.10-pocketsphinx" with "gstreamer1.0-pocketsphinx"
#	Added "pip install --upgrade pip"
#	Replaced "openjdk-7-jdk" with "openjdk-8-jdk"

set -e

# Prelims
cd ${HOME}

sudo apt-get update

sudo apt-get --yes install python-dev python-opencv python-wxgtk3.0 python-pip python-matplotlib python-pygame python-lxml

sudo pip install --upgrade pip

sudo apt-get --yes install python python-all-dev python-pip build-essential swig git
sudo apt-get --yes install portaudio19-dev 
sudo apt-get --yes install libpulse-dev
sudo apt-get --yes install python-gst-1.0
sudo apt-get --yes install gstreamer1.0-pocketsphinx

sudo pip install future
sudo pip install pyglet 

sudo apt-get --yes install chromium-browser
sudo apt-get --yes install filezilla
sudo apt-get --yes install geany
sudo apt-get --yes install gimp
sudo apt-get --yes install meld
sudo apt-get --yes install kazam


sudo apt-get update



# Install espeak
cd ${HOME}
sudo apt-get --yes install espeak
sudo apt-get --yes install mbrola
sudo apt-get --yes install mbrola-us1
sudo apt-get --yes install mbrola-us2
# When giving commands in the terminal window:
# espeak –v mb-us1 –s 180 –p 30 “<insert text to be read aloud>”


sudo apt-get update





# 1) ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	
sudo apt-get update

sudo apt-get --yes install ros-kinetic-desktop-full

sudo rosdep init
rosdep update
	
echo "" >> ${HOME}/.bashrc
echo "# Set ROS Environment Variables:" >> ${HOME}/.bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ${HOME}/.bashrc
source /opt/ros/kinetic/setup.bash
source ${HOME}/.bashrc

sudo apt-get --yes install python-rosinstall
		
# Create catkin workspace:
mkdir -p ${HOME}/catkin_ws/src
	
# Build the workspace:
cd ${HOME}/catkin_ws
catkin_make	

# Edit .bashrc again:
# Set ROS Environment Variables:
echo "source ~/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc
source ${HOME}/catkin_ws/devel/setup.bash
source ${HOME}/.bashrc		



# 2) Rosbridge
cd ${HOME}
sudo apt-get --yes install ros-kinetic-rosbridge-suite



# 3) pymavlink
echo "" >> ${HOME}/.bashrc
echo "# Add mavlink to PYTHONPATH" >> ${HOME}/.bashrc
echo "export PYTHONPATH=\${PYTHONPATH}:\$HOME/mavlink" >> ${HOME}/.bashrc
source ${HOME}/.bashrc

cd ${HOME}
git clone git://github.com/optimatorlab/mavlink.git --recursive
cd ${HOME}/mavlink/pymavlink
sudo python setup.py install



# 4) MAVproxy 
#	https://github.com/Dronecode/MAVProxy
#	http://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html
# OLD: sudo pip install MAVProxy
cd ${HOME}
git clone https://github.com/optimatorlab/MAVProxy
cd ${HOME}/MAVProxy
sudo python setup.py install
		
sudo adduser ${USER} dialout

# If you have trouble connecting Pixhawk, try the following:
sudo usermod -a -G dialout ${USER}



# 5) Dronekit
# http://python.dronekit.io/guide/quick_start.html
# OLD:  sudo pip install dronekit	
cd ${HOME}
git clone https://github.com/optimatorlab/dronekit-python
cd ${HOME}/dronekit-python
sudo python setup.py build
sudo python setup.py install
				
# sudo pip install dronekit-sitl
cd ${HOME}
git clone https://github.com/optimatorlab/dronekit-sitl
cd ${HOME}/dronekit-sitl
sudo python setup.py build
sudo python setup.py install



# ===============================================================================================
# 6a) Cesium -- Master Branch:
cd ${HOME}
git clone -b master --single-branch git://github.com/optimatorlab/cesium cesium

sudo apt-get update

cd ${HOME}/cesium
curl -sL https://deb.nodesource.com/setup_4.x | sudo -E bash -
sudo apt-get install -y nodejs
npm install

cd ${HOME}/cesium
git clone https://github.com/optimatorlab/Chart.js.git
# This will save a directory named ~/cesium/Chart.js
npm install chart.js --save 
# This will install/save to ~/cesium/node_modules/chart.js
rm -r ${HOME}/cesium/Chart.js

cd ${HOME}/cesium
git clone https://github.com/optimatorlab/dials_gauges.git


# 6b) Cesium -- 3D Tiles Branch:  https://github.com/optimatorlab/cesium/tree/3d-tiles
#cd ${HOME}
#git clone -b 3d-tiles --single-branch git://github.com/optimatorlab/cesium cesium-3d-tiles
#
#cd ${HOME}/cesium-3d-tiles
#npm install
#npm run build
#npm run minifyRelease
#
#cd ${HOME}/cesium-3d-tiles
#git clone https://github.com/optimatorlab/Chart.js.git
## This will save a directory named ~/cesium-3d-tiles/Chart.js
#npm install chart.js --save 
## This will install/save to ~/cesium-3d-tiles/node_modules/chart.js
#rm -r ${HOME}/cesium-3d-tiles/Chart.js
#
#cd ${HOME}/cesium-3d-tiles
#git clone https://github.com/optimatorlab/dials_gauges.git

# ===============================================================================================



# 7) jMAVsim
# Install Java SDK (v1.7.0_95):
sudo apt-get install openjdk-8-jdk

# Install ant (v1.9.3):
sudo apt-get install ant

# Get jMAVSim
cd ${HOME} 
git clone https://github.com/optimatorlab/jMAVSim
cd ${HOME}/jMAVSim
git submodule init
git submodule update
				
# Compile jMAVSim:
cd ${HOME}/jMAVSim
ant

# modemmanager interferes with any non-modems and should be removed.
sudo apt-get remove modemmanager 

# 8) QGroundControl
#
#
#	FIXME!!!!
#
#
#


# 9) Speech Recognition
# Install pyaudio
cd ${HOME}
git clone https://github.com/optimatorlab/pyaudio.git
cd ${HOME}/pyaudio
sudo python setup.py install

# Install pocketsphinx
cd ${HOME}
sudo pip install pocketsphinx

# Install SpeechRecognition 3.4.6
cd ${HOME}
sudo pip install SpeechRecognition


# Blather -- Originally from https://github.com/ajbogh/blather
cd ${HOME}
git clone https://github.com/optimatorlab/blather.git

mkdir ${HOME}/.config/blather
mkdir ${HOME}/.config/blather/language
mkdir ${HOME}/.config/blather/plugins

cd ${HOME}/blather/config/blather
mv ${HOME}/blather/config/blather/commands.conf ${HOME}/.config/blather/commands.conf
mv ${HOME}/blather/config/blather/sentences.corpus ${HOME}/.config/blather/sentences.corpus

cd ${HOME}/blather/config/blather/language
mv ${HOME}/blather/config/blather/language/lm ${HOME}/.config/blather/language/lm
mv ${HOME}/blather/config/blather/language/dic ${HOME}/.config/blather/language/dic

cd ${HOME}/blather/config/blather/plugins
mv ${HOME}/blather/config/blather/plugins/thunderbird.sh ${HOME}/.config/blather/plugins/thunderbird.sh



echo "Installation Complete."
echo "Reboot your machine before running the software."
