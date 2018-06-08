# treeseg

Extract individual trees from lidar point clouds

<img src="https://4xnnka.bl3302.livefilestore.com/y4mzpqPKgclbQKR51t4a9tA2v37IKMp221dDuLXHvRqQzc8szMleP7anmHxoy-bPwa-0qnDRrRpQkBhkO2P3cef-u7_eBpNubE_wO8FA5E0sDkbmPt0BIJoWT2upX_YFO5UfTS0ys0LZLHaSl0DktwdJIj82R4ZP051-2CiPzJAXJvht3eijlwfp0mo_W8VeXVa8p_g79_EEb51Kg0WGMWe8A?width=660&height=628&cropmode=none" width="500">

## Overview

treeseg has been developed to near-automatically extract tree-level point clouds from high-density larger-area lidar point clouds acquired in forested scenes.
A full description of the methods can be found here: http://discovery.ucl.ac.uk/1575534/ (pp. 96-167)

Briefly, libtreeseg provides a set of generic functions that have been wrapped here into a series of executables to: i) identify individual stems in the larger-area point cloud (findstems), ii) extract each of these identified stems up to the first order of branching (segmentstem), and iii) segment each individual crown from neighbouring vegetation (segmentcrown).

Integrated into treeseg is libleafsep, a library providing methods to separate wood and leaf returns in unorganised point clouds. 
libleafsep is a modified translation of TLSeparation (https://github.com/TLSeparation).

## Prerequisites

treeseg has been developed using:

* Point Cloud Library (PCL) (http://pointclouds.org) (v1.7 or later)
* Armadillo (http://arma.sourceforge.net) (v7.960 or later)

On RHEL 7, these dependencies can be installed via:

```
yum install cmake
yum install pcl pcl-devel pcl-tools pcl-doc
yum install armadillo armadillo-devel
```

On Ubuntu:

```
apt-get install -y git cmake
apt-get install -y libpcl-dev
apt-get install -y libproj-dev
```

On macOS 10.13, dependencies are perhaps most easily installed via Homebrew (https://brew.sh):

```
brew install cmake
brew install pcl
brew install armadillo
```

## Optional, but useful software:
* [CloudCompare] (http://www.cloudcompare.org/): for visualizing outputs


## Installation

Install libtreeseg and the associated binaries as follows:

```
git clone https://github.com/apburt/treeseg.git;
cd treeseg;
mkdir build;
cd build;
cmake ../src;
make;
```

Also included in treeseg is rxp2pcd, for conversion of REIGL V-Line scan data to .pcd binary format. Linux-only, this requires the RiVLIB headers and libraries (downloadable from: http://www.riegl.com/index.php?id=224) to be installed in /treeseg/include/riegl/ and /treeseg/lib/ respectively.


## Usage

Below is an example usage of the treeseg binaries:

* plotcoords ../matrix/ > nouragesH20_coords.dat
* rxp2pcd ../ nouraguesH20_coords.dat 60 15 nouraguesH20
* nearestneighbour 1 4 nouraguesH20.sample.pcd > nouraguesH20_nn.dat
* downsample 0.04 1 nouraguesH20_*.pcd
* getdemslice 2 3 6 nouraguesH20_*.downsample.pcd
* findstems 15 0.2 2 ../nouraguesH20_coords.dat ../nouraguesH20.slice.downsample.pcd
* segmentstem 12.5 ../nouraguesH20.downsample.pcd ../clusters/cluster_*.pcd
* getcrownvolume ../nouraguesH20.downsample.pcd ../stems/stem_*.pcd
* segmentcrown [14 - 16] ../volume_*.pcd

## Singularity Usage
An easier way to get treeseg working is to use [Singularity containers] (https://singularity.lbl.gov/index.html) (even on Windows!).  You won't be limited by your OS or have to compile software.  The [University of Nevada, Reno GEARS Lab] (https://naes.unr.edu/gears) is maintaining treeseg images and other utility images.  Install Singularity on your OS of choice following that website.  It is worth quickly reviewing the [QuickStart] (https://singularity.lbl.gov/quickstart) to familiarize yourself with treeseg. To get the core containers for using treeseg (once Singularity is installed):

```
# For Windows/Mac, you will do these from within the vagrant image you created.
# Drive mapping is a bit weird, so read up on those.
# Set where you want the containers to live, e.g. your home directory:
export SINGULARITY_PULLFOLDER=~

# Download the "stable" treeseg image:
singularity pull shub://gearslaboratory/gears-singularity:treeseg
# Download the PDAL image to use more starting file formats (e.g. LAS):
singularity pull shub://gearslaboratory/gears-singularity:gears-pdal

```
Notes on images:

* There are likely some development images in our [Singularity Hub repository] (https://www.singularity-hub.org/collections/428) as well

Now is time to run treeseg.  We are going to assume the data is in LAS format (exportable from just about every system).  When running singularity from Mac/Windows, you'll basically be inside a virtual Ubuntu via vagrant so these commands will likely work:  

```
### Set some variables:
# Input file info:
LASINPUT="/pathto/exported/TLS_scan.las"

# File basename to use for naming:
LASBASE=$(basename "$LASINPUT" .las)

# Now set up working directory.  For now we'll use a temp folder:
TREESEG_DIR=$(mktemp -p $LASBASE"_treeseg" -d)
cd $TREESEG_DIR

# Here are key parameters:
# Voxel size for downsampling (in point cloud units, e.g. meters).
# Burt used 0.04 meters, Calders often used 0.05 meters.
LEAF_SIZE="0.04"

# TODO: OTHER PARAMETERS:





```


## Authors

* **Andrew Burt**
* **Mathias Disney**
* **Kim Calders**
* **Matheus Boni Vicari**
* **With tweaks from: Jonathan Greenberg**


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgements

* treeseg makes use of the Point Cloud Library (PCL) (http://pointclouds.org) and Armadillo (http://arma.sourceforge.net).
* AB was supported by NERC CASE PhD NE/J016926/1 with EADS Astrium; additional funding was via the NERC National Centre for Earth Observation (NCEO), NERC funding awards NE/N00373X/1, NE/P011780/1 and NE/K002554/1; MD is also supported by the EU Horizon2020 project (BACI project funded by the EU's Horizon 2020 Research and Innovation Programme under grant agreement 640176).
