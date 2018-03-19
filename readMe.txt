Andrew Burt
a.burt.12@ucl.ac.uk
01/05/2015

treeseg
--------------------------

Aims:

	- Extract individual trees
	- Optimise and run qsm_tree v2.0
	- Perform analysis on qsm's
	- Simulate VZ-400
	
Does not:

	1. Registration of indiviual scans on plot coordiate system (requires RIEGL RiSCAN Pro) 
	2. Construct structual models from tree point clouds (requires Pasi Raumonen qsm_tree) 

--------------------------

Created using:

	PCL - pointclouds.org
	RiVLIB/RiWaveLIB - riegl.com
	librat - librat.wikispaces.com

--------------------------

Caveats:

	Tested on RIEGL VZ-400 only

--------------------------

Requirements:

	UNIX-like OS (Linux for full compatability)
	C/C++ compiler
	Python(v2.7.x) (numpy+scipy)
	MATLAB(2015b)
	PCL(v1.7.2)
		-OS X: Homebrew
		-Ubuntu: apt-get			
		-RedHat: yum

-------------------------

Build:

	CMakeLists.txt:

		cmake ../src
		make

	vz-400_basic.c:

		gcc -L/home/plewis/bpms/src/lib -I/home/plewis/bpms/src/lib -I/home/plewis/bpms/src/lib/rat -I/data/TLS/tools/treeseg/include /data/TLS/tools/treeseg/src/vz-400_basic.c -o vz-400_basic -lmatrix_x86_64 -lalloc_x86_64 -lvect_x86_64 -lerr_x86_64 -lhipl_x86_64 -lrand_x86_64 -limage_x86_64 -lrat_x86_64 -lm
	

--------------------------

qsm_tree modifications:

	Add directories to MATLAB path:

		addpath(genpath('/Users/ucfaab0/Dropbox/UCL/MATLAB/'));
		addpath(genpath('/Users/ucfaab0/Dropbox/UCL/treeseg/MATLAB'));
		savepath;

	The following modifications within Pasi Raumonen's cylinder_tree_model are required:

		cover_sets.m

			line 57: np = size(P,1);
			
			to:	 np = size(P,1);
				 s = RandStream.create('mt19937ar','seed',sum(100*clock));
				 RandStream.setGlobalStream(s);

		tree_sets.m && cylinders.m

			RUNTIME = ???;
			timerID = tic;
			while
				if(toc(timerID) > RUNTIME)
					break;
				end
			end

		tree_data.m

			Remove all triangulation components

--------------------------

Description:

	downsample.cpp:

		downsample [edge_length] [splits] [cloud] 

			Generate voxel grid of [edge_length] over [cloud] and aggregate entity contents into single point. [splits] specifed to overcome indices overflow. Downsampled cloud named downsample.pcd.

	getstems.cpp:

		getstems [cluster_spatial_tolerance] [cluster_minsize] [vector_angular_tolerance] [bounding_box_expansion] [min_dbh] [max_variationcoefficient] [cloud]

			Generate clusters conforming to [cluster_spatial_tolerance] and [cluster_minsize] over [cloud]. Principle Eigenvector derived per cluster covariance matrix overlaid clusters output as slice_clusters_pca.pcd. Stems identified if normal (+/- [vector_angular_tolerance]) to ground vector (here assumed [stem_x,stem_y,0]) after concatenation of duplicates through intersection testing of gently expanded bounding boxes [bounding_box_expansion]. Second level of testing involves multiple 2D RANSAC circle fitting along extent of cloud. cluster_*.pcd written if stems exceed [minimum_dbh] and are within [max_variationcoefficient] derived from the mean and coefficient of variation from the multiple circle fit. 

	isolateskeleton.cpp:

		isolateskeleton [max_distance_between_clouds] [cloud(s)]

			Generate clusters of points over volume cloud [cloud] where the clustering specifications are based upon the nearest-neighbour relationsips over z-slices, the principle Eigenvector is then derived from the covariance matrix of each cluster, the clusters overlaid with the principle eigenvector are written to tree_?_clusters_pca.pcd. Next, a cluster-correction is undertaken using a region-growing approach to breakup the clusters by surface features identified by the normals. The clusters overlaid with their newly derived principle eigenvector are written to tree_?_clusters_corrected.pcd. Finally a skeleton of the tree is created initialsised by identifying the base of the trunk - the nearest 20 neighbours to the trunk are then assesed for connectivity based upon their spatial attributes, the angle between the two principle eigenvectors and their magnitude. Every new connection undertakes the same approach until the skeleton has been created and written to tree_?.pcd.

	pcdASCII2binary.cpp

		pcdASCII2binary [cloud(s)]

			Convert PCD [clouds(s]) from ASCII to binary. Clobbers input file(s).

	pcdbinary2ASCII.cpp

		pcdbinary2ASCII [cloud(s)]

			Convert PCD [cloud(s)] from binary to ASCII. Clobbers input file(s).

	removegroundcanopy.cpp:

		removegroundcanopy [resolution] [z_min] [z_max] [incloud]

			Generate DEM (output as dem.txt) at [resolution] over [incloud] and generate slice from [z_min] to [z_max] named slice.pcd.

	rxp2pcd.cpp (LINUX only - requires RIEGL libscanifc-mt.so):

		rxp2pcd [x_min] [x_max] [y_min] [y_max] [reflectance_threshold] [deviation_threshold]

			Output plot cloud constrained to coordinates [x_min][x_max][y_min][y_max], thresholded to reflectance and deviations of [reflectance_threshold] and [deiviation_threshold]. Working directory must be top level of raw scan directory i.e. .riproject containing ScanPos001,ScanPos002 etc alongside folder named matrix containing .dat SOP exports in 001.dat,002.dat convention.

	segmentvolume.cpp:

		segmentvolume [crown_extent_factor] [height_factor] [plot_cloud] [stem_cloud(s)]

			Implement 2D RANSAC circle fit on [stem_clous(s)] to approximate stem diameter. Estimated DBH drives 2 pass-through filters on [plot_cloud]. The first extracts the isolated stem and the second the canopy based on [crown_extent_factor] and [height_factor] producing volume_*.pcd 

----------------------------

Examples:

	Plot: nouraguesH20, caxiuanaA, lope01, gold0101, kara001, kara005
	Sampling: 10m sq, 20m sq, 20m sq, 50m star, 50m star, 50m star
	Resolution: 0.04, 0.04, 0.06, 0.06, 0.06, 0.06

	1) plotcoords ../matrix/ > nouraguesH20_coords.dat
		      ../matrix/ > caxiuanaA_coords.dat
		      ../matrix/ > lope01_coords.dat
		      ../matrix/ > gold0101_coords.dat
		      ../matrix/ > kara001_coords.dat
		      ../matrix/ > kara005_coords.dat

	2) rxp2pcd ../ nouraguesH20_coords.dat 60 10 nouraguesH20
		   ../ caxiuanaA_coords.dat 60 10 caxiuanaA
		   ../ lope01_coords.dat 60 10 lope01
	           ../ gold0101_coords.dat 15 10 gold0101
		   ../ kara001_coords.dat 15 10 kara001
		   ../ kara005_coords.dat 15 10 kara005

	3) nearestneighbour 1 4 nouraguesH20.sample.pcd > nouraguesH20_nn.dat
			    1 4 caxiuanaA.sample.pcd > caxiuanaA_nn.dat
			    1 4 lope01.sample.pcd > lope01_nn.dat
			    1 4 gold0101.sample.pcd > gold0101_nn.dat
			    1 4 kara001.sample.pcd > kara001_nn.dat
			    1 4 kara005.sample.pcd > kara005_nn.dat

	4) downsample 0 0.05 nouraguesH20_*.pcd
		      0 0.05 caxiuanaA_*.pcd
		      0 0.05 lope01_*.pcd
		      0 0.05 gold0101_*.pcd
		      0 0.05 kara001_*.pcd
		      0 0.05 kara005_*.pcd
	
	5) getslice 2 3 6 nouraguesH20.downsample.pcd
		    2 3 6 caxiuanaA.downsample.pcd
	            2 3 6 lope01.downsample.pcd
	   	    2 1 3 gold0101.downsample.pcd
	   	    2 1 3 kara001.downsample.pcd
	   	    2 1 3 kara005.downsample.pcd

	6) getclusters ../nouraguesH20_coords.dat 15 0.2 2 ../nouraguesH20.slice.downsample.pcd
		       ../caxiuanaA_coords.dat 20 0.2 2 ../caxiuanaA.slice.downsample.pcd
		       ../lope01_coords.dat 20 0.2 2 ../lope01.slice.downsample.pcd
		       ../gold0101_coords.dat 15 0.2 2 ../gold0101.slice.downsample.pcd
		       ../kara001_coords.dat 15 0.2 2 ../kara001.slice.downsample.pcd
		       ../kara005_coords.dat 15 0.2 2 ../kara005.slice.downsample.pcd

	7) segmentstems ../nouraguesH20.downsample.pcd 22.5 ../clusters/cluster_*.pcd
			../caxiuanaA.downsample.pcd 27.5 ../clusters/cluster_*.pcd
			../lope01.downsample.pcd 27.5 ../clusters/cluster_*.pcd
			../gold0101.downsample.pcd 27.5 ../clusters/cluster_*.pcd
			../kara001.downsample.pcd 27.5 ../clusters/cluster_*.pcd
			../kara005.downsample.pcd 27.5 ../clusters/cluster_*.pcd

	***manually check/filter with sortStems.py***

	8) isolatetree ../nouraguesH20.downsample.pcd 22.5 nouraguesH20 ../stems/stem_*.pcd
		       ../caxiuanaA.downsample.pcd 27.5 caxiuanaA ../stems/stem_*.pcd
		       ../lope01.downsample.pcd 27.5 lope01 ../stems/stem_*.pcd
		       ../gold0101.downsample.pcd 27.5 gold0101 ../stems/stem_*.pcd
		       ../kara001.downsample.pcd 27.5 kara001 ../stems/stem_*.pcd
		       ../kara005.downsample.pcd 27.5 kara005 ../stems/stem_*.pcd

	***manually check/filter with sortTrees.py***

	9) rxp2tree ../ 0.5 ../extraction/trees/nouraguesH20_*.pcd
		    ../ 0.5 ../extraction/trees/caxiuanaA_*.pcd
		    ../ 0.5 ../extraction/trees/lope01_*.pcd
		    ../ 0.5 ../extraction/trees/gold0101_*.pcd
		    ../ 0.5 ../extraction/trees/kara001_*.pcd
		    ../ 0.5 ../extraction/trees/kara005_*.pcd

	10) cloudresults 60 0.33 ./clouds/*.txt > control_cloud.results

	11) runCylinderModel.py -i ../clouds/control_*.xyz -p N -m /usr/local/bin/matlab

	12) optimiseCylinderModel -p N
