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
	pico .bashrc
	```
	
	Add the following lines at the end of `.bashrc`, changing the Gurobi path as appropriate:

	```
	export GUROBI_HOME="/opt/gurobi751/linux64"
	export PATH="${PATH}:${GUROBI_HOME}/bin"
	export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
	```

4. Obtain a free academic license:

	- https://user.gurobi.com/download/licenses/free-academic

5. Activate your Gurobi license, replacing the x's below with your license key:

	```
	grbgetkey xxxxxxxxxxxxxxxxxxxxx
	```

6. Create the `gurobipy` package, so we can import within Python.  Change the Gurobi path as appropriate:

	```
	cd /opt/gurobi751/linux64
	sudo python setup.py install
	```
	
7. Test:

	```
	python
	>>> import gurobipy
	```
	
8. (optional)  If you're using Python virtual environments, create a symbolic link from within a particular virtualenv:

	```
	workon <virtualenv_name>
	cd ~/.virtualenvs/olab2/lib/python2.7/site-packages/		# Change path for your virtualenv
	ln -s /opt/gurobi751/linux64/lib/python2.7/gurobipy.so		# might need to change path for your version
	```
