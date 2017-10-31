# Manual Installation Instructions

This document explains how to manually install the software required for our HITL and SITL simulators.

* Updated 10/31/2017 by Chase Murray *

#### NOTES:
- We are using Ubuntu 14.04 in the lab.  Sometime "soon" we should look to upgrade to 16.04.
- IF YOU EXPERIENCE ISSUES WITH THESE INSTRUCTIONS,
	  PLEASE DOCUMENT THE ERRORS YOU'RE GETTING AND CONTACT THE DOCUMENT OWNER.
	  WE WANT THIS GUIDE TO BE UPDATED AND ACCURATE.

#### SUMMARY OF REQUIRED SOFTWARE TO BE INSTALLED:
1. ROS Indigo Igloo
2. Rosbridge
3. pymavlink
4. MAVproxy
5. Dronekit
6. Cesium 	
7. jMAVsim
8. QGroundControl (allows loading firmware onto the UAVs)
9. Speech Recognition and Text-to-Speech


## 0) These are nice to have on your machine:
Open a terminal and enter the following commands:
```
sudo apt-get install chromium-browser
sudo apt-get install filezilla
sudo apt-get install geany
sudo apt-get install gimp
sudo apt-get install meld
sudo apt-get install kazam
```

You may also want to download the Slack desktop client:
[https://slack.com/downloads/linux](https://slack.com/downloads/linux)

## 1) ROS Indigo Igloo.
(Note:  Jade and Kinetic will probably be fine, but UB is using Indigo.)

These steps are mostly copied from http://wiki.ros.org/indigo/Installation/Ubuntu

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	
sudo apt-get update

sudo apt-get install ros-indigo-desktop-full

sudo rosdep init
rosdep update
	
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall
```
	
#### Create catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src	
```
	
#### Build the workspace:
```
cd ~/catkin_ws/
catkin_make	
```

#### Edit .bashrc to make sure the following 3 lines are there:

> \# Set ROS Environment Variables:

> source /opt/ros/indigo/setup.bash

> source ~/catkin_ws/devel/setup.bash

```
pico ~/.bashrc	
source ~/.bashrc
```

## 2) Install Rosbridge:

```
cd ~
sudo apt-get install ros-indigo-rosbridge-suite
```

#### Double-check that 

> source /opt/ros/indigo/setup.bash

is in ~/.bashrc.  (it should've been added when ROS was installed)


## 3) pymavlink:

- See http://qgroundcontrol.org/mavlink/pymavlink

#### Add the following 2 lines to .bashrc:

> \# Add mavlink to PYTHONPATH

> export PYTHONPATH=${PYTHONPATH}:$HOME/mavlink

```
source .bashrc
```

#### Mavlink is now using some Python 3 stuff.  Need to install `future`:
```
sudo apt-get update
sudo apt-get install python-pip
sudo pip install future
```

#### Now, install pymavlink:
```
cd ~
git clone git://github.com/optimatorlab/mavlink.git --recursive	
cd ~/mavlink/pymavlink
sudo python setup.py install
```

###### NOTE: We have forked from git://github.com/mavlink/mavlink.git

## 4) MAVproxy: 

- See https://github.com/Dronecode/MAVProxy
- See http://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html

```
sudo apt-get install python-dev python-opencv python-wxgtk2.8 python-pip python-matplotlib python-pygame python-lxml

cd ~
git clone https://github.com/optimatorlab/MAVProxy
cd ~/MAVProxy
sudo python setup.py install

sudo adduser ${USER} dialout
```

###### NOTE:  Before forking to our repository, we used `sudo pip install MAVProxy`


## 5) Dronekit:

- See http://python.dronekit.io/guide/quick_start.html

```
sudo apt-get install python-pip python-dev

cd ~
git clone https://github.com/optimatorlab/dronekit-python
cd ~/dronekit-python
sudo python setup.py install

cd ~
git clone https://github.com/optimatorlab/dronekit-sitl
cd ~/dronekit-sitl
sudo python setup.py install
```

###### NOTE: Before forking to our repository, we used `sudo pip install dronekit` and `sudo pip install dronekit-sitl`
	

## 6) Cesium:
1. If `~/cesium` already exists, rename it to something else (as a backup)
	
2. Choose **ONE** of the following approaches:
    1.  **FOR THE LATEST RELEASE:**

```
cd ~
mkdir cesium
```
		
- Go to http://cesiumjs.org/tutorials/cesium-up-and-running/
- In the "Downloading a Cesium release" section, click on the blue button titled Cesium-(version).zip ...choose to open that file in archive manager
- In archive manager, extract that .zip file into your new cesium directory which you made in step (1) ...make sure to select "extract all"		

    2. **FOR THE LAB'S VERSION (1.38):**

```
cd ${HOME}/cesium
wget https://github.com/AnalyticalGraphicsInc/cesium/releases/download/1.38/Cesium-1.38.zip
unzip Cesium-1.38.zip
```

3. In a new terminal, issue the following command to update linux:

```
sudo apt-get update
```
	
4. In a new terminal, change directories to ~/cesium:

```
cd ~/cesium/
```
	
5. Issue the following command to download node (note that this will only work if you are in the ~/cesium directory):

```
curl -sL https://deb.nodesource.com/setup_4.x | sudo -E bash -
```
	
6. Issue the following command (note that this will only work if you are in the ~/cesium directory):

```
sudo apt-get install -y nodejs
```
	
7. Execute the following command (note that this will only work if you are in the ~/cesium directory).  This will create a folder named 'node_modules'. 

```
npm install
```
	
8. Install should be complete. To check if you can open a server:

```
cd ~/cesium
node server.js
```
	
If it tells you that a server is running locally, everything is running properly. Close out of the server by hitting `Ctrl-C`

9. Install Chart.js

```
cd ${HOME}/cesium
git clone https://github.com/optimatorlab/Chart.js.git
# This will save a directory named ~/cesium/Chart.js
npm install chart.js --save 
# This will install/save to ~/cesium/node_modules/chart.js
rm -r ${HOME}/cesium/Chart.js
```

10.  Install Dials/Gauges

```
cd ${HOME}/cesium
git clone https://github.com/optimatorlab/dials_gauges.git
```


## 7) jMAVsim

a. Install Java SDK (v1.7.0_95):

```
sudo apt-get install openjdk-7-jdk
```

b. Install ant (v1.9.3):

```
sudo apt-get install ant
```

c. Get jMAVSim -- (Originally from https://github.com/DrTon/jMAVSim)

```
cd ~ 
git clone https://github.com/optimatorlab/jMAVSim
cd jMAVSim
git submodule init
git submodule update
```

d.  **NOTE:  STEP (d) IS NO LONGER REQUIRED.**  I'm leaving it here just in case we need to revisit this in the future.

	> \# Edit jMAVSim Configs: 
	> cd ~/jMAVSim/src/me/drton/jmavsim
	> Open “Simulator.java” 
	> Find this line of code in “Simulator.java”:
	> 	serialMAVLinkPort.open("/dev/tty.usbmodem1", 230400, 8, 1, 0);
	> Change to:
	> 	serialMAVLinkPort.open("/dev/ttyACM0", 230400, 8, 1, 0);
	>
	> Also, for debugging purposes, you might want to turn on the default 3D viz environment.
	> Find the "if (false) {" line and change to true.
	> In the future we'll add a command line flag to toggle this option.
	>			
	> You may also change the default starting location (GPS coords). 
				
e. Compile jMAVSim:

```
cd ~/jMAVSim
ant
```

f. modemmanager interferes with any non-modems and should be removed.

```
sudo apt-get remove modemmanager 
```	
g.  If you have trouble connecting Pixhawk, try the following:

```
sudo usermod -a -G dialout $USER
```	

## 8) QGroundControl

#### This software allows us to load firmware onto the UAVs.

- Visit https://github.com/mavlink/qgroundcontrol/releases/
- Download version 2.9.3
- Extract --\> will create "qgroundcontrol" directory in home directory.

Run QGroundControl using `./qgroundcontrol-start.sh`


## 9) Speech Recognition and Text-to-Speech

#### Install portaudio19-dev and the python development package (python-all-dev) beforehand. 
```
cd ~
sudo apt-get install portaudio19-dev python-all-dev
```

#### Install libpulse-dev
```
cd ~
sudo apt-get install libpulse-dev
```

#### Install pyaudio -- Originally from http://people.csail.mit.edu/hubert/git/pyaudio.git
```
cd ~
git clone https://github.com/optimatorlab/pyaudio.git
cd ~/pyaudio
sudo python setup.py install
```

#### Install pocketsphinx
```
cd ~
sudo apt-get install python python-all-dev python-pip build-essential swig git
sudo pip install pocketsphinx
```

#### Install SpeechRecognition 3.4.6
```
sudo pip install SpeechRecognition
```

#### Try out these examples
```
cd ~/pyaudio/examples
python microphone_recognition.py
python audio_transcribe.py
```

#### I can't remember where we use this.  Install it anyway:
```
sudo pip install pyglet 
```


## Blather
These instructions are modified from https://github.com/ajbogh/blather:

```
cd ~
sudo apt-get install python-gst0.10
sudo apt-get install gstreamer0.10-pocketsphinx

cd ~
git clone https://github.com/optimatorlab/blather.git
```


#### OPTION 1:  Configure your environment manually:
1. Move `~/blather/commands.tmp` to `~/.config/blather/commands.conf` and fill the file with sentences and commands to run:

```
mkdir ~/.config/blather/
mv ~/blather/commands.tmp ~/.config/blather/commands.conf
pico ~/.config/blather/commands.conf
```

2. Run `Blather.py`, this will generate `~/.config/blather/sentences.corpus` based on sentences in the `commands.conf` file:

```
cd ~/blather
python Blather.py
```

3. Quit blather (there is a good chance it will just segfault)
    
4. Go to http://www.speech.cs.cmu.edu/tools/lmtool.html and upload the `sentences.corpus` file:

    - See ~/.config/blather/sentences.corpus
    - This is in a hidden directory, making it difficult to find it for uploading.

```    
cp ~/.config/blather/sentences.corpus ~/Desktop/sentences.corpus
```

    - You may delete ~/Desktop/sentences.corpus after you've uploaded it.
	
5. Download the resulting XXXX.lm file to the desktop, and save the file as lm.txt
    
6. Download the resulting XXXX.dic file to the desktop, and save the file as dic.txt
    
7. make a backup of the old lm/dic files:

```
cd ~/.config/blather/language
mv lm lm.bak
mv dic dic.bak
```
		
8. Move (and rename, with no extension) the downloaded files to the language subdirectory:
```
mv ~/Desktop/lm.txt ~/.config/blather/language/lm
mv ~/Desktop/dic.txt ~/.config/blather/language/dic
```
		
9. Run Blather.py:
```		
cd ~/blather
./Blather.py
```		

say "hello world"
    
    - Notes from the code originator:
        - for Qt GUI, run Blather.py -i q
        - for Gtk GUI, run Blather.py -i g
        - to start a UI in 'continuous' listen mode, use the -c flag 
        - to use a microphone other than the system default, use the -d flag
        - Once the sentences.corpus file has been created, run the language_updater.sh script to automate the process of creating and downloading language files.	

#### OPTION 2:  Use a standard configuration we created on github:
```
mkdir ~/.config/blather
mkdir ~/.config/blather/language
mkdir ~/.config/blather/plugins

cd ~/blather/config/blather
mv commands.conf ~/.config/blather/commands.conf
mv sentences.corpus ~/.config/blather/sentences.corpus

cd ~/blather/config/blather/language
mv lm ~/.config/blather/language/lm
mv dic ~/.config/blather/language/dic

cd ~/blather/config/blather/plugins
mv thunderbird.sh ~/.config/blather/plugins/thunderbird.sh
```	
	
	


## espeak -- Text-to-speech

#### Install Espeak:
```
sudo apt-get install espeak
```

Or, do it manually:
- Open Ubuntu Software Center
- Search “Espeak”
- Download the first option (Multi-lingual software speech synthesizer)


#### Install Mbrola:
```
sudo apt-get install mbrola
sudo apt-get install mbrola-us1
sudo apt-get install mbrola-us2
```

Or, do it manually:
- Open Ubuntu Software Center
- Search “Mbrola”
- Download the first and only option (Multilingual software speech synthesizer)
- Search "mbrola-us1" and install
- Search "mbrola-us2" and install

	
When giving commands in the terminal window:

```
espeak –v mb-us1 –s 180 –p 30 “this text will be read aloud”
```	
