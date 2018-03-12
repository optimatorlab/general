# Using and Installing pgRouting

*Updated March 12, 2018*


### Overview

This guide assumes that you already have pgRouting configured on your system.  *This is the case for the workstations in our lab.*

If you do *not* have pgRouting configured, go to "PART II -- Installing and Configuring pgRouting" at the bottom to see the details.

There are two parts to this guide:

1. **Examples** -- Assuming that your machine has already been configured for pgRouting, this section provides some examples for downloading OSM data and storing it in your database.

2. **Installing and Configuring pgRouting Databases** -- This part is for reference only.  Student workstations in the lab already have all of the necessary software installed and configured.  

---

- References: 
	- http://trac.osgeo.org/postgis/wiki/UsersWikiPostGIS22UbuntuPGSQL95Apt
	- https://github.com/pgRouting/pgrouting/wiki/Download%3A-Debian-and-Ubuntu 
	- http://workshop.pgrouting.org/2.1.0-dev/en/chapters/prepare_data.html

---

## PART I -- Some pgRouting Examples

We'll look at two examples.  The first example comes from a pgRouting workshop.  The second example creates a database of Buffalo streets.

### Example 1 -- pgRouting Workshop

1. Create a database named `city_routing` and connect to it:

	```
	psql -U user
	CREATE DATABASE city_routing;
	\c city_routing
	```
	
	- Result:

		```
		psql (9.6.1, server 9.5.5)
		You are now connected to database "city_routing" as user "user".
		```

2. Create `postgis` and `pgrouting` extensions:
	
	```
	CREATE EXTENSION postgis;
	CREATE EXTENSION pgrouting;
	\dx+ pgRouting
	SELECT pgr_version();
	```

3. Download workshop data:

	```
	mkdir ~/Projects/pgData
	cd ~/Projects/pgData
	CITY="BONN_DE"
	wget -N --progress=dot:mega "http://download.osgeo.org/livedvd/data/osm/$CITY/$CITY.osm.bz2"
	bunzip2 $CITY.osm.bz2
	```

4. Convert the Open Street Map (OSM) data to a format suitable for pgRouting:
	
	```
	cd ~/Projects/pgData
	osm2pgrouting     -f BONN_DE.osm     -d city_routing     -U user
	```
	
	- Check the tables:

		```
		psql -U user -d city_routing -c "\d"
		```

		You should see a list of tables.


5. Let's access the database named `city_routing` as the user `user`:

	```
	psql -U user city_routing
	```
	
	- You should see the following:

		```
		psql (9.6.1, server 9.5.5)
		Type "help" for help.
	
		city_routing=#
		```
		
	- You have logged into the database `city_routing`. To see if it's running properly, we'll try the following example queries and check their results:

	- **Example 1:**
	
		```
		SELECT * FROM pgr_dijkstra('SELECT gid AS id, source, target, length AS cost FROM ways', 13224, 6549, directed := false);
		```
		
		Result:
		
		```
		 seq | path_seq | node  | edge  |         cost         |      agg_cost       
		-----+----------+-------+-------+----------------------+---------------------
		   1 |        1 | 13224 |   815 |  0.00177810584893137 |                   0
		   2 |        2 |  3495 |   820 |  0.00179225967147818 | 0.00177810584893137
		   3 |        3 |   905 | 19177 | 0.000102092605022171 | 0.00357036552040955
		   4 |        4 |  6893 | 19178 | 8.66225143958747e-05 | 0.00367245812543172
		   5 |        5 |  7753 | 19191 |  0.00047705096163826 |  0.0037590806398276
		   6 |        6 |  8159 |  7015 | 0.000513853140499986 | 0.00423613160146586
		   7 |        7 |  2477 |  7016 | 9.12382595170489e-05 | 0.00474998474196585
		   8 |        8 | 11902 |  7017 | 9.71560600269149e-05 | 0.00484122300148289
		   9 |        9 |  2042 |  7018 | 0.000120258430056735 | 0.00493837906150981
		  10 |       10 |  4004 |  7019 |  0.00102611053011065 | 0.00505863749156655
		  11 |       11 |  6549 |    -1 |                    0 | 0.00608474802167719
		(11 rows)
		```

	- **Example 2:**

		```
		SELECT * FROM pgr_dijkstraCost('SELECT gid AS id, source, target, length_m  / 1.3 / 60 AS cost FROM ways', ARRAY[6549, 1458, 9224], ARRAY[13224, 6963], directed := false) ORDER BY end_vid;
		```
		
		Result:
		
		```
		 start_vid | end_vid |     agg_cost     
		-----------+---------+------------------
		      1458 |    6963 | 13.5539128131556
		      6549 |    6963 | 8.34274572465808
		      9224 |    6963 | 19.3719411554243
		      1458 |   13224 | 19.9557289926127
		      6549 |   13224 | 6.63986000245047
		      9224 |   13224 | 31.9456044752323
		(6 rows)
		```

	- **Example 3:**

		```
		select * from ways where source = 2456;
		```

		Result:
		
		```
		gid  | class_id |        length        |     length_m     | name | source | target |    x1     |     y1     |    x2     |     y2     |         cost         |     reverse_cost     |      cost_s       |  reverse_cost_s   | rule | one_way | maxspeed_forward | maxspeed_backward |  osm_id   | source_osm | target_osm | priority |                                          the_geom                                          
		-------+----------+----------------------+------------------+------+--------+--------+-----------+------------+-----------+------------+----------------------+----------------------+-------------------+-------------------+------+---------+------------------+-------------------+-----------+------------+------------+----------+--------------------------------------------------------------------------------------------
		16181 |      122 | 3.16829607199584e-05 | 2.31649176854942 |      |   2456 |   3293 | 7.1508158 | 50.7297039 | 7.1507849 | 50.7296969 | 3.16829607199584e-05 | 3.16829607199584e-05 | 0.178191674503802 | 0.178191674503802 |      |       0 |               50 |                50 | 165250111 | 1768070017 | 1768070020 |        1 | 0102000020E6100000020000005CE102756F9A1C40BF21F9EF665D49402AEC585B679A1C400DBF40B5665D4940
		(1 row)
		```

---

### Example 2 -- Make a Database with Buffalo Data:

1. Connect to the `user` DB:

	```
	psql -U user
	```
	
2. From the psql prompt:

	```
	CREATE DATABASE buffalo_routing;
	\c buffalo_routing
	CREATE EXTENSION postgis;
	CREATE EXTENSION pgrouting;
	\q
	```
	
3. Make the `~/Projects/pgData` directory (if it doesn't already exist), and `cd` into it:

	```
	mkdir ~/Projects/pgData
	cd ~/Projects/pgData
	```
	
5. Download data for the Buffalo region:

	```
	CITY="BUFFALO_US"
	BBOX="-78.9086,42.7858,-78.6388,43.0197"
	wget --progress=dot:mega -O "$CITY.osm" "http://www.overpass-api.de/api/xapi?*[bbox=${BBOX}][@meta]"
	```
	
6. Run the converter:

	```
	osm2pgrouting     -f BUFFALO_US.osm     -d buffalo_routing     -U user
	```
	
7. Check that tables exist:
	
	```
	psql -U user -d buffalo_routing -c "\d"
	```






---

--- 
	
## PART II -- Installing and Configuring pgRouting

<FONT COLOR=red><B>NOTE:</B> If you are a student in the lab, the installation and configuration of pgRouting has already been performed.  Thus, this section is for reference only.  In other words, you do not need to perform any steps in the remainder of this document.</FONT>


1. Add repository to `sources.list`:

	- Find out the codename for your version of Ubuntu.  For example, Ubuntu 14.04 is `trusty`.  Ubuntu 16.04 is `xenial`.  If you're unsure, try:

	```
	lsb_release -a 
	```

	- Automated:
		
		```
		sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt/ $(lsb_release -cs)-pgdg main" > /etc/apt/sources.list.d/pgdg.list'
		# sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt/ $(lsb_release -cs)-pgdg main" > /etc/apt/sources.list'
		```
		

	- For **14.04 (trusty)** only:

		```
		sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt/ trusty-pgdg main" >> /etc/apt/sources.list'
		```
	
	- For **16.04 (xenial)** only:	
		
		```
		sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt xenial-pgdg main" >> /etc/apt/sources.list'
		```
	

2. Add keys:

	```	
	wget --quiet -O - http://apt.postgresql.org/pub/repos/apt/ACCC4CF8.asc | sudo apt-key add -
	```
	
	```
	sudo apt-get update
	```

	- Verify that things worked so far (replace `xenial` with `trusty` if you're on 14.04):
	
		```
		cd /var/lib/apt/lists
		ls apt.postgresql.org_pub_repos_apt_dists_xenial-pgdg_*
		```
		
		Should return:
		
		```
		apt.postgresql.org_pub_repos_apt_dists_xenial-pgdg_InRelease                   
		apt.postgresql.org_pub_repos_apt_dists_xenial-pgdg_main_binary-i386_Packages
		apt.postgresql.org_pub_repos_apt_dists_xenial-pgdg_main_binary-amd64_Packages
		```

3. Install `postgresql 9.5`, `PostGIS 2.2`, `PGAdmin3`, `pgRouting 2.1` and additional supplied modules including the `adminpack` extension:

	```
	sudo apt-get install postgresql-9.5-postgis-2.2 pgadmin3 postgresql-contrib-9.5
	```

	- You should get a result like this:
	
		```
		Setting up postgresql-client-common (177.pgdg14.04+1) ...
		Setting up postgresql-client-9.5 (9.5.5-1.pgdg14.04+1) ...
		update-alternatives: using /usr/share/postgresql/9.5/man/man1/psql.1.gz to provide /usr/share/man/man1/psql.1.gz (psql.1.gz) in auto mode
		Setting up postgresql-common (177.pgdg14.04+1) ...
		Adding user postgres to group ssl-cert
		
		Creating config file /etc/postgresql-common/createcluster.conf with new version
		
		Creating config file /etc/logrotate.d/postgresql-common with new version
		Building PostgreSQL dictionaries from installed myspell/hunspell packages...
		  en_au
		  en_gb
		  en_us
		  en_za
		Removing obsolete dictionary files:
		 * No PostgreSQL clusters exist; see "man pg_createcluster"
		Setting up postgresql-9.5-postgis-2.3-scripts (2.3.1+dfsg-1.pgdg14.04+1) ...
		Setting up postgresql-client-9.6 (9.6.1-2.pgdg14.04+1) ...
		update-alternatives: using /usr/share/postgresql/9.6/man/man1/psql.1.gz to provide /usr/share/man/man1/psql.1.gz (psql.1.gz) in auto mode
		Setting up postgresql-client (9.6+177.pgdg14.04+1) ...
		Setting up sysstat (10.2.0-1) ...
		
		Creating config file /etc/default/sysstat with new version
		update-alternatives: using /usr/bin/sar.sysstat to provide /usr/bin/sar (sar) in auto mode
		Setting up proj-bin (4.8.0-2ubuntu2) ...
		Processing triggers for ureadahead (0.100.0-16) ...
		Setting up postgresql-9.5 (9.5.5-1.pgdg14.04+1) ...
		Creating new cluster 9.5/main ...
		  config /etc/postgresql/9.5/main
		  data   /var/lib/postgresql/9.5/main
		  locale en_US.UTF-8
		  socket /var/run/postgresql
		  port   5432
		update-alternatives: using /usr/share/postgresql/9.5/man/man1/postmaster.1.gz to provide /usr/share/man/man1/postmaster.1.gz (postmaster.1.gz) in auto mode
		 * Starting PostgreSQL 9.5 database server                                                                                                                                                            [ OK ] 
		Setting up postgresql-contrib-9.5 (9.5.5-1.pgdg14.04+1) ...
		Setting up odbcinst (2.2.14p2-5ubuntu5) ...
		Setting up odbcinst1debian2:amd64 (2.2.14p2-5ubuntu5) ...
		Setting up libgdal1h (1.10.1+dfsg-5ubuntu1) ...
		Setting up postgresql-9.5-postgis-2.2 (2.2.2+dfsg-7~137.gite11228b.pgdg14.04+1) ...
		Setting up postgresql-9.5-postgis-2.3 (2.3.1+dfsg-1.pgdg14.04+1) ...
		Processing triggers for libc-bin (2.19-0ubuntu6.9) ...
		```

4. Install `pgRouting`: 

	```
	sudo apt-get install postgresql-9.5-pgrouting
	```

5. Install `osm2pgrouting`:

	```
	sudo apt-add-repository -y ppa:georepublic/pgrouting
	sudo apt-get update
	sudo apt-get install osm2pgrouting
	```

---

### Preparing the Database

In this section, we configure our database.  This only needs to happen once.  

**NOTE**:  Students in the lab do NOT need to perform these steps...your workstation has already been properly configured.
	
- Source: http://workshop.pgrouting.org/2.1.0-dev/en/chapters/prepare_data.html


1. Open `psql` as user `postgres`:

	```
	sudo -u postgres psql
	```
	
2. Change the password for user `postgres` and exit `psql`:

	```
	ALTER USER postgres PASSWORD 'postgres';
	```
	
	```
	\q
	```

3. Make a backup of your `pg_hba.conf` file:

	```
	cd /etc/postgresql/9.5/main
	sudo cp pg_hba.conf pg_hba.conf.bak
	```
	
4. Edit `pg_hba.conf`:

	```
	sudo nano /etc/postgresql/9.5/main/pg_hba.conf
	```
	
	- Change the authentication method for `postgres` to `md5` and for `all` to `trust`.  This will remove the password requirement from all the users *except* `postgres`. 
	
	- Also, comment out the credentials related to `IPv4` and `IPv6`, and use a common connection for them, with address `localhost` and method `trust`.
	
	- Your file should now look like this:

		```
		# Database administrative login by Unix domain socket
		local   all             postgres                                md5
	
		# TYPE  DATABASE        USER            ADDRESS                 METHOD
	
		# "local" is for Unix domain socket connections only
		local   all             all                                     trust
		host    all             all             localhost               trust
		# IPv4 local connections:
		#host    all             all             127.0.0.1/32            md5
		# IPv6 local connections:
		#host    all             all             ::1/128                 md5
		# Allow replication connections from localhost, by a user with the
		# replication privilege.
		#local   replication     postgres                                peer
		#host    replication     postgres        127.0.0.1/32            md5
		#host    replication     postgres        ::1/128                 md5
		```
		
	- More information about `pg_hba.conf` can be found here: https://www.postgresql.org/docs/9.1/static/auth-pg-hba-conf.html


5. Restart the database service:

	```
	sudo service postgresql restart
	```
	
6. Create a "user" role on psql:

	```
	psql -U postgres
	```
	
	- Input password: `postgres`
	
	- Within psql, create a "user" role:

		```
		CREATE ROLE "user" SUPERUSER LOGIN;
		```
		
		**FIXME!**  Do we need to issue the following command?  `ALTER USER "user" PASSWORD 'user';`

		```
		\q
		```

7. At this point, a role named "user" is created, but there is no database named "user". So, when we write `psql -U user`, it tries to connect to DB "user" (which doesn't exist) through role "user". 
	
	Instead of doing this, write `psql -U user postgres`. This will connect to DB "postgres" through role "user".  To avoid writing `psql -U user postgres` everytime we want to login through "user" in the future, we will create a DB named "user". After that, we can write `psql -U user` to login into DB "user" through role "user".

	```
	psql -U user -d postgres
	CREATE DATABASE "user";
	\q
	```
