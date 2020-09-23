#!/bin/bash

# ===============================================================================
# File:  auto_install_1804_minimal.sh
#
# Updated 9/23/2020 by Chase Murray
#
# NOTES:
#	* This script installs the "base" software for students in the lab running
#	  Ubuntu 18.04. 
#	* It also installs ROS Melodic on Ubuntu 18.04.
#
# INSTALLATION INSTRUCTIONS: 
#	1) Download this script to the location of your choice.  ~/Downloads is fine.
#	2) Change directories to the location where this script is saved.
#	3) chmod +x auto_install_1804.sh
#	4) ./auto_install_1804.sh
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
#	10/31/17:
#		No longer using cesium-3d-tiles
#		Pulling Cesium directly from their releases page.
#	12/22/17:
#		Added pyserial.  Should be v3.4 or above now.
#	2/2/18:
#		Added -H flag for pip installs
#   3/10/18:
#		Added Python virtual environments
#		Removed anything involving "sudo" and "pip".  Pip should be run at the user level within a virtualenv.
#			One exception:  We sudo pip to install virtualenv and virtualenvwrapper	
#		Added pgRouting installation
#		Modified version of Cesium -- FIXME
#	3/16/18:
#		Added R/RStudio
#	3/26/18:
# 		Added retext, OCR, TeXstudio, Sublime Text, Chrome, and QGIS
#	3/28/18:
#		Hard-coded xenial (replaced lsb_release)
#	3/29/18:
#		Fixed QGIS installation.
#		Fixed psql installation.
#	8/20/18:
#		Using latest version of node.js
#		pymavlink is no longer available from mavlink repo.  
#			I tried using ArduPilot repo (forked to our repo), but that failed.
#			Now using sudo pip.
#		Added installation of osrm-backend
#	10/31/19:
#		Updated some keyring info
#		Removed --yes options from (some of the) installations
#		Updated version of Cesium to 1.62
#	3/20/20:
#		Transitioned to 18.04 (bionic)
#		Updated version of Cesium to 1.67
#		Updated ROS to Melodic
#		Using lsb_release 
#			(instead of hard-coding xenial (16.04) or bionic (18.04))	
#

set -e

# Store the present working directory.
# We'll use this later when copying psql config file.
myPWD=${PWD}

# Create the user's "Projects" directory:
mkdir -p ${HOME}/Projects

# Copy wallpaper to a new "olab" directory:
mkdir -p ${HOME}/olab
cp ${myPWD}/optimator_lab_transparency_small.png ${HOME}/olab/optimator_lab_transparency_small.png

# Prelims
cd ${HOME}

sudo apt-get update
sudo apt-get upgrade


# --------------------------------------
# Added 3/10/18, Updated 3/20/20:
# These are for openCV.  See https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/ 
sudo apt-get --yes install cmake unzip pkg-config
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python3-dev

# sudo apt-get install libtiff5-dev 

# UNABLE TO FIND  
# sudo apt-get install libjasper-dev 
# I think this is for openCV

# UNABLE TO FIND
# sudo apt-get install libpng12-dev

# sudo apt-get install libgtk-3-dev
# ??? sudo apt-get install libhdf5-serial-dev
# ??? sudo apt-get install graphviz
# ??? sudo apt-get install libopenblas-dev

sudo apt-get install python3-tk

sudo apt-get install python-tk
sudo apt-get install python-imaging-tk
sudo apt-get install python2.7-dev
sudo apt-get install python-qt4
# --------------------------------------

# SHOULD WE INCLUDE ALL OF THE FOLLOWING?
# Or, should we be using pip for these?
# sudo apt-get --yes install python-dev python-opencv python-wxgtk3.0 python-pip python-matplotlib python-pygame python-lxml
sudo apt-get install python-dev python-wxgtk3.0 python-lxml

# SHOULD WE INCLUDE ALL OF THE FOLLOWING?
# Or, should we be using pip for these?
sudo apt-get --yes install python python-all-dev build-essential swig git
sudo apt-get --yes install portaudio19-dev 
sudo apt-get --yes install libpulse-dev
sudo apt-get --yes install python-gst-1.0
sudo apt-get --yes install gstreamer1.0-pocketsphinx

# Popular applications for students in the lab:
sudo apt-get --yes install chromium-browser
sudo apt-get --yes install filezilla
sudo apt-get --yes install geany
sudo apt-get --yes install gimp
sudo apt-get --yes install meld
sudo apt-get --yes install kazam
sudo apt-get --yes install retext

# Python OCR
sudo apt-get --yes install tesseract-ocr

# TeXstudio
# Open with "texstudio"
# IGNORED FOR MINIMAL INSTALLATION
# sudo apt-get update
# sudo apt-get --yes install texlive-full
# sudo apt-get --yes install texstudio

# Sublime text
# https://www.sublimetext.com/docs/3/linux_repositories.html
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get --yes install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get --yes install sublime-text

# Chrome
# https://askubuntu.com/questions/79280/how-to-install-chrome-browser-properly-via-command-line
sudo apt-get --yes install libxss1 libappindicator1 libindicator7
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo dpkg -i google-chrome*.deb
# If error messages pop up after running the command sudo dpkg -i google-chrome*.deb then run the command
# sudo apt-get install -f
# and then re-run the previous command. The error messages mentioned should include something similar to

# QGIS
# https://qgis.org/en/site/forusers/alldownloads.html#debian-ubuntu
# IGNORED FOR MINIMAL INSTALLATION
# sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 51F523511C7028C3
# echo "deb https://qgis.org/debian bionic main" | sudo tee -a /etc/apt/sources.list
# echo "deb-src https://qgis.org/debian bionic main" | sudo tee -a /etc/apt/sources.list
# sudo apt-get update
# sudo apt-get install qgis python-qgis qgis-plugin-grass

# R and RStudio:
# See https://www.digitalocean.com/community/tutorials/how-to-install-r-on-ubuntu-18-04
# IGNORED FOR MINIMAL INSTALLATION
# sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys E298A3A825C0D65DFD57CBB651716619E084DAB9
# sudo add-apt-repository 'deb https://cloud.r-project.org/bin/linux/ubuntu bionic-cran35/'
# sudo apt update
# sudo apt install r-base


# FIXME:  This is for 18.04 (bionic) only.
# See https://www.rstudio.com/products/rstudio/download/#download for the latest versions of RStudio.
# IGNORED FOR MINIMAL INSTALLATION
# wget https://download1.rstudio.org/desktop/bionic/amd64/rstudio-1.2.5033-amd64.deb
# sudo apt-get install gdebi-core
# sudo gdebi -n rstudio-1.2.5033-amd64.deb
# rm rstudio-1.2.5033-amd64.deb

# FIXME -- Should we remove (uninstall) old versions of pip?
sudo apt-get purge --auto-remove python-pip	

# Install pip:
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
sudo python3 get-pip.py

# Install virtualenv and virtualenvwrapper:
sudo pip install virtualenv virtualenvwrapper
sudo rm -rf ${HOME}/.cache/pip get-pip.py


# Configure virtualenv and virtualenvwrapper in your .bashrc profile:
echo "" >> ${HOME}/.bashrc
echo "# Configure virtualenv and virtualenvwrapper:" >> ${HOME}/.bashrc
echo "export WORKON_HOME=$HOME/.virtualenvs" >> ${HOME}/.bashrc
echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3" >> ${HOME}/.bashrc
echo "source /usr/local/bin/virtualenvwrapper.sh" >> ${HOME}/.bashrc
source ${HOME}/.bashrc

sudo apt-get clean

# -----------------------------------------
# 1) ROS Melodic
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	
sudo apt-get update

sudo apt-get install ros-melodic-desktop-full

# This is new.  Not sure why rosdep wasn't previously installed?
sudo apt install python-rosdep

sudo rosdep init
rosdep update
	
echo "" >> ${HOME}/.bashrc
echo "# Set ROS Environment Variables:" >> ${HOME}/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ${HOME}/.bashrc
source /opt/ros/melodic/setup.bash
source ${HOME}/.bashrc

sudo apt-get install python-rosinstall
		
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

sudo apt-get clean

# -----------------------------------------
# 2) Rosbridge
cd ${HOME}
sudo apt-get install ros-melodic-rosbridge-suite


# -----------------------------------------
# 3) pymavlink
# REMOVED 8/20/18:
# echo "" >> ${HOME}/.bashrc
# echo "# Add mavlink to PYTHONPATH" >> ${HOME}/.bashrc
# echo "export PYTHONPATH=\${PYTHONPATH}:\$HOME/mavlink" >> ${HOME}/.bashrc
# source ${HOME}/.bashrc

sudo apt-get --yes install python-future

# REMOVED 8/20/18:
# cd ${HOME}
# sudo rm -rf ${HOME}/mavlink
# git clone git://github.com/optimatorlab/mavlink.git --recursive
# cd ${HOME}/mavlink/pymavlink
# sudo python setup.py install

# TRIED 8/20/18, but FAILED:
# cd ${HOME}
# sudo rm -rf ${HOME}/pymavlink
# git clone git://github.com/optimatorlab/pymavlink.git --recursive
# cd ${HOME}/pymavlink
# sudo python setup.py install

# Added 8/20/18:
sudo pip2 install -U pymavlink

# Added 3/20/20:
sudo pip3 install -U pymavlink

sudo apt-get clean

# -----------------------------------------
# 4) MAVproxy 
#	https://github.com/Dronecode/MAVProxy
#	http://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html
# OLD: sudo pip install MAVProxy
cd ${HOME}
sudo rm -rf ${HOME}/MAVProxy
git clone https://github.com/optimatorlab/MAVProxy
cd ${HOME}/MAVProxy
sudo python setup.py install
# Added 3/20/20:
sudo python3 setup.py install
		
sudo adduser ${USER} dialout

# If you have trouble connecting Pixhawk, try the following:
sudo usermod -a -G dialout ${USER}

sudo apt-get clean

# -----------------------------------------
# 5) Dronekit
# http://python.dronekit.io/guide/quick_start.html
# OLD:  sudo pip install dronekit	
cd ${HOME}
sudo rm -rf ${HOME}/dronekit-python
git clone https://github.com/optimatorlab/dronekit-python
cd ${HOME}/dronekit-python
sudo python setup.py build
sudo python setup.py install
				
# OLD:  sudo pip install dronekit-sitl
cd ${HOME}
sudo rm -rf ${HOME}/dronekit-sitl
git clone https://github.com/optimatorlab/dronekit-sitl
cd ${HOME}/dronekit-sitl
sudo python setup.py build
sudo python setup.py install
# Added 3/20/20:
sudo python3 setup.py build
sudo python3 setup.py install


sudo apt-get clean

# -----------------------------------------
# 6) Cesium -- 1.69 (Released May 2, 2020):
# Delete the cesium directory (if it exists)
sudo rm -rf ${HOME}/cesium

# Copy/install cesium:
mkdir ${HOME}/cesium
cd ${HOME}/cesium
wget https://github.com/AnalyticalGraphicsInc/cesium/releases/download/1.69/Cesium-1.69.zip
unzip Cesium-1.69.zip
rm Cesium-1.69.zip

sudo apt-get update

cd ${HOME}/cesium
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
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


sudo apt-get clean

# -----------------------------------------
# 7) jMAVsim
# Install Java SDK (v1.7.0_95):
sudo apt-get --yes install openjdk-8-jdk

# Install ant (v1.9.3):
sudo apt-get --yes install ant

# Get jMAVSim
cd ${HOME} 
sudo rm -rf ${HOME}/jMAVSim
git clone https://github.com/optimatorlab/jMAVSim
cd ${HOME}/jMAVSim
# Removed 3/20/20:
# git submodule init
# git submodule update

# Compile jMAVSim:
cd ${HOME}/jMAVSim
ant

# modemmanager interferes with any non-modems and should be removed.
sudo apt-get remove modemmanager 


# -----------------------------------------
# 8) QGroundControl
#
#
#	FIXME!!!!
#
#
#


# -----------------------------------------
# 9) APM Mission Planner
# FIXME
# See http://ardupilot.org/planner2/docs/installation-for-linux.html


# -----------------------------------------
# 10) pgRouting
# IGNORED FOR MINIMAL INSTALLATION


# ---------------------------------
# 11) Speech Recognition
sudo apt-get update

# Install espeak
cd ${HOME}
sudo apt-get --yes install espeak
sudo apt-get --yes install mbrola
sudo apt-get --yes install mbrola-us1
sudo apt-get --yes install mbrola-us2
# When giving commands in the terminal window:
# espeak –v mb-us1 –s 180 –p 30 “<insert text to be read aloud>”


# Install pyaudio
cd ${HOME}
sudo rm -rf ${HOME}/pyaudio
git clone https://github.com/optimatorlab/pyaudio.git
cd ${HOME}/pyaudio
sudo python setup.py install
# Added 3/20/20:
sudo python3 setup.py install

# Install pocketsphinx
# OLD:
# 	cd ${HOME}
# 	sudo -H pip install pocketsphinx
# NEW:
#	pip install pocketsphinx 		-- from within virtualenv

# Install SpeechRecognition 3.4.6
# OLD:
#	cd ${HOME}
# 	sudo -H pip install SpeechRecognition
# NEW:
#	pip install SpeechRecognition	-- from within virtualenv


# Blather -- Originally from https://github.com/ajbogh/blather
cd ${HOME}
sudo rm -rf ${HOME}/blather
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


# --------------------------------------------
# Added 8/20/18
# OSRM Backend
# https://github.com/Project-OSRM/osrm-backend/wiki/Running-OSRM
# https://www.digitalocean.com/community/tutorials/how-to-set-up-an-osrm-server-on-ubuntu-14-04
# IGNORED FOR MINIMAL INSTALLATION

# --------------------------------------------
# Added 5/8/20
# ORS Local
# These instructions come from https://docs.docker.com/engine/install/ubuntu/.
# Also see https://github.com/optimatorlab/veroviz/blob/master/ORS_install_instructions.md
# IGNORED FOR MINIMAL INSTALLATION


# --------------------------------------------
# Added 5/8/20
# Allows use of right click with touchpad
# Once installed, type "tweaks" in the application menu.
sudo apt install gnome-tweaks

echo "Installation Complete."
echo "Reboot your machine before running the software."

