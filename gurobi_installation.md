# Install Gurobi

1. Download:

	- Visit https://www.gurobi.com/downloads/gurobi-optimizer

	- Select `64-bit Linux Current` version and download the compressed distribution file.

2. Extract the archive to `/opt` using the following command, changing the Gurobi version as appropriate:

	```
	sudo tar xvfz ~/Downloads/gurobi7.5.1_linux64.tar.gz -C /opt
	```
	
	After extraction, make note of where Gurobi was installed.  
	It should be something like `/opt/gurobi701/linux64/`.  
	We'll need this path in the steps below.

3. Update `.bashrc` to include Gurobi environment variables:

	```
	cd ~
	pico .bashrc
	```
	
	Add the following lines at the end of `.bashrc`, changing the Gurobi path as appropriate:

	```
	export GUROBI_HOME="/opt/gurobi751/linux64"
	export PATH="${PATH}:${GUROBI_HOME}/bin"
	export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
	```

	Reload the `.bashrc` file:
	
	```
	source .bashrc
	```
	
4. Obtain a free academic license:

	- https://user.gurobi.com/download/licenses/free-academic

5. Activate your Gurobi license, replacing the x's below with your license key:

	```
	grbgetkey xxxxxxxxxxxxxxxxxxxxx
	```

6. Create the `gurobipy` package, so we can import within Python.  

	- First, let's make sure we aren't in a Python virtual environment:
	
		```
		deactivate
		```
		
	- Now, build the `gurobipy` package.  Change the Gurobi path below as appropriate:

		```
		cd /opt/gurobi751/linux64
		sudo python setup.py install
		```
	
7. Test:

	```
	python
	>>> import gurobipy
	```
	
8. (*optional*)  If you're using Python virtual environments...

	- To use `gurobipy` within a virtual environment, we'll need to create a symbolic link to the `gurobipy.so` file.
	
	- The code below assumes that (1) we have a Python 2.7 virtual environment named `olab2` with a particular path, and (2) we have installed Gurobi 7.5.1 and have found the path to `gurobipy.so`.  **You may need to change these paths as appropriate for your setup.**
	
		```
		workon olab2
		cd ~/.virtualenvs/olab2/lib/python2.7/site-packages 
		ln -s /opt/gurobi751/linux64/lib/python2.7/gurobipy.so
		```
