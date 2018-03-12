# general
Our first repository

## Versions:
- ROS Kinetic Kame on Ubuntu 16.04
- (deprecated) ROS Indigo Igloo on Ubuntu 14.04

---

## Automated Installation Instructions:

1. This assumes that you have already installed Ubuntu 16.04.  Visit https://www.ubuntu.com/download for installation instructions.

2. You'll need `git` to download the installation package:

	```
	sudo apt-get install git
	```

3. Clone this repository into your `Downloads` directory.

	```
	cd ~/Downloads
	rm -rf general
	git clone https://github.com/optimatorlab/general
	```

4. Run the auto-installer to configure our "base" Ubuntu 16.04 installation:

	```
	cd ~/Downloads/general
	chmod +x auto_install_1604.sh
	./auto_install_1604.sh
	```

	- If/when prompted to "remove write-protected regular files", you may enter `Y`.

## Install Additional Packages that can't be Automated:

The following code should also be on your machine, but it couldn't be automated within the `auto_install_1604.sh` script.

1. XCTU

	See https://www.digi.com/resources/documentation/Digidocs/90001110-88/tasks/t_download_and_install_xctu_linux.htm

	```
	# This step is included in the auto installer.
	# I'm leaving it here just in case:
	# sudo usermod -a -G dialout ${USER}
	```

	```
	cd ~/Downloads
	mkdir XCTU
	cd XCTU
	wget "https://www.digi.com/xctu-linux-x64"
	chmod +x xc*
	./xctu
	```
	
	Follow the install wizard

		- Installation Directory:  `/home/<username>/Digi` (replace <username> with your name

	```
	rm -rf ~/Downloads/XCTU
	```
	
2. Gurobi
	
	- See [Gurobi Installation Instructions](gurobi_installation.md).
	
3. Slack 
	
	- Visit https://slack.com/downloads/linux and follow the installation instructions. 
		
4. Concorde	

	- These steps are adapted from http://www.math.uwaterloo.ca/tsp/concorde/DOC/README.html

	```	
	cd ~
	rm -rf concorde
	wget "http://www.math.uwaterloo.ca/tsp/concorde/downloads/codes/src/co031219.tgz"
	gunzip co031219.tgz 
	tar xvf co031219.tar
	cd concorde 
	./configure
	rm ~/co031219.tar
	```

	**DOES THIS STILL WORK?  WHAT ELSE DID WE DO?**	


5. APM Planner -- See http://ardupilot.org/planner2/docs/installation-for-linux.html

		```
		cd ~/Downloads
		mkdir APM
		cd APM
		wget "http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner_2.0.24_xenial64.deb"
		```

		```	
		sudo dpkg -i apm_planner*.deb
		```
		
		If the installation fails due to missing dependencies:
		
		```
		sudo apt-get -f install
		```	

		```
		sudo dpkg -i apm_planner*.deb
		```

		```
		rm -rf ~/Downloads/APM
		apmplanner2
		```


6. Install Cesium Addons

	- First, clone the `cesium-addons` repository.  This requires that you have access to that repository.
		
		```
		cd ~/Downloads
		rm -rf ~/Downloads/cesium-addons
		git clone https://github.com/optimatorlab/cesium-addons.git
		```

	- Next, install the `cesium-addons` code:
		
		```
		cd ~/Downloads/cesium-addons
		chmod +x cesium-addons_install.sh
		./cesium-addons_install.sh
		cd ~/Downloads
		rm -rf ~/Downloads/cesium-addons
		```		


---
 
## Notes
- As of 3/12/18, the [manual installation instructions](manual_install.md) are deprecated.  I'm leaving the link here just for reference.
- As of 3/12/18, the [Ubuntu 14.04 installation script](auto_install_1404.sh) is deprecated.  Again, the link is left here just for reference.

---

## Troubleshooting

- WebGL on Chromium:
    - Open Chromium
    - Type chrome://flags in the address bar
    - CTRL-f and type "rendering list".  "Override software rendering list" will come up.
    - Click "Enable" and relaunch the browser (relaunch button appears at bottom of browser).
