#!/bin/bash

# ===============================================================================
# File:  auto_install_1604.sh
#
# Updated 3/12/2018 by Chase Murray
#
# NOTES:
#	* This script installs the "base" software for students in the lab running
#	  Ubuntu 16.04. 
#	* It also installs ROS Kinetic on Ubuntu 16.04.
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


set -e

# Store the present working directory.
# We'll use this later when copying psql config file.
myPWD=$PWD


# Prelims
cd ${HOME}

sudo apt-get update
sudo apt-get upgrade

# --------------------------------------
# Added 3/10/18:
sudo apt-get --yes install cmake unzip pkg-config
sudo apt-get --yes install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get --yes install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get --yes install libxvidcore-dev libx264-dev
sudo apt-get --yes install libgtk-3-dev
sudo apt-get --yes install libhdf5-serial-dev graphviz
sudo apt-get --yes install libopenblas-dev libatlas-base-dev gfortran
sudo apt-get --yes install python-tk python3-tk python-imaging-tk
sudo apt-get --yes install python2.7-dev python3-dev
sudo apt-get --yes install python-qt4
# --------------------------------------

# SHOULD WE INCLUDE ALL OF THE FOLLOWING?
# Or, should we be using pip for these?
# sudo apt-get --yes install python-dev python-opencv python-wxgtk3.0 python-pip python-matplotlib python-pygame python-lxml
sudo apt-get --yes install python-dev python-wxgtk3.0 python-lxml

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

sudo apt-get update


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


# -----------------------------------------
# 1) ROS Kinetic
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


# -----------------------------------------
# 2) Rosbridge
cd ${HOME}
sudo apt-get --yes install ros-kinetic-rosbridge-suite


# -----------------------------------------
# 3) pymavlink
echo "" >> ${HOME}/.bashrc
echo "# Add mavlink to PYTHONPATH" >> ${HOME}/.bashrc
echo "export PYTHONPATH=\${PYTHONPATH}:\$HOME/mavlink" >> ${HOME}/.bashrc
source ${HOME}/.bashrc

cd ${HOME}
git clone git://github.com/optimatorlab/mavlink.git --recursive
cd ${HOME}/mavlink/pymavlink
sudo python setup.py install


# -----------------------------------------
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


# -----------------------------------------
# 5) Dronekit
# http://python.dronekit.io/guide/quick_start.html
# OLD:  sudo pip install dronekit	
cd ${HOME}
git clone https://github.com/optimatorlab/dronekit-python
cd ${HOME}/dronekit-python
sudo python setup.py build
sudo python setup.py install
				
# OLD:  sudo pip install dronekit-sitl
cd ${HOME}
git clone https://github.com/optimatorlab/dronekit-sitl
cd ${HOME}/dronekit-sitl
sudo python setup.py build
sudo python setup.py install



# -----------------------------------------
# 6) Cesium -- 1.38 (Released Oct. 2, 2017):
# FIXME -- 1.43 is now available (Released March 2018)
# Delete the cesium directory (if it exists)
rm -rf ${HOME}/cesium

# Copy/install cesium:
mkdir ${HOME}/cesium
cd ${HOME}/cesium
#wget https://github.com/AnalyticalGraphicsInc/cesium/releases/download/1.38/Cesium-1.38.zip
#unzip Cesium-1.38.zip
wget https://github.com/AnalyticalGraphicsInc/cesium/releases/download/1.43/Cesium-1.43.zip
unzip Cesium-1.43.zip

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


# ===============================================================================================
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


# -----------------------------------------
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

# FIXME -- NOT SURE WHICH FILE IS CORRECT:
sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt/ $(lsb_release -cs)-pgdg main" > /etc/apt/sources.list.d/pgdg.list'
# sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt/ $(lsb_release -cs)-pgdg main" > /etc/apt/sources.list'

wget --quiet -O - http://apt.postgresql.org/pub/repos/apt/ACCC4CF8.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install postgresql-9.5-postgis-2.2 pgadmin3 postgresql-contrib-9.5
sudo apt-get install postgresql-9.5-pgrouting

sudo apt-add-repository -y ppa:georepublic/pgrouting
sudo apt-get update
sudo apt-get install osm2pgrouting

# Set password:
sudo -u postgres psql -c "ALTER USER postgres PASSWORD 'postgres';"

# Make a backup of `pg_hba.conf`:
sudo mv /etc/postgresql/9.5/main/pg_hba.conf /etc/postgresql/9.5/main/pg_hba.conf.bak

# Replace `pg_hba.conf` with the version downloaded from github:
# FIXME -- UPLOAD A CONFIG FILE FOR 9.5
sudo cp ${myPWD}/pg_hba.conf.template /etc/postgresql/9.5/main/pg_hba.conf

# Ensure proper permissions and ownership:
# -rw-r----- 1 postgres postgres  4651 Nov 27  2016 pg_hba.conf
sudo chown postgres:postgres /etc/postgresql/9.5/main/pg_hba.conf
sudo chmod 0640 /etc/postgresql/9.5/main/pg_hba.conf

# Restart the database service:
sudo service postgresql restart
sleep 5s

# Create a "user" role on psql:
# hostname:port:database:username:password
echo "localhost:*:postgres:postgress" >> ${HOME}/.pgpass
chmod 0600 ${HOME}/.pgpass
psql -U postgres -w -c "CREATE ROLE 'user' SUPERUSER LOGIN;"
rm ${HOME}/.pgpass

# At this point, a role named "user" is created, but there is no database named "user". So, when we write `psql -U user`, it tries to connect to DB "user" (which doesn't exist) through role "user". 
#
# Instead of doing this, write `psql -U user postgres`. This will connect to DB "postgres" through role "user".  To avoid writing `psql -U user postgres` everytime we want to login through "user" in the future, we will create a DB named "user". After that, we can write `psql -U user` to login into DB "user" through role "user".
psql -U user -d postgres -c "CREATE DATABASE 'user';"
#	\q			FIXME -- DO WE NEED THIS?



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
git clone https://github.com/optimatorlab/pyaudio.git
cd ${HOME}/pyaudio
sudo python setup.py install


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
