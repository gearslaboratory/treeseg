//Andrew Burt - a.burt@ucl.ac.uk

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "boost/program_options.hpp"
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <treeseg.h>

namespace
{
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t SUCCESS = 0;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;

} // namespace

int main (int argc, char *argv[])
{
	// Trying to get command line args working:
	namespace po = boost::program_options;
	po::options_description desc("Options");
	desc.add_options()
		("help,h", "Print help messages")
		("verbose,v", "Execute verbosely")
		("downsample", po::value<std::string>()(&downsample)->required(), "downsample file!")
		("resolution", po::value<std::float>()(&resolution)->2, "resolution!")
		("zmin", po::value<std::float>()(&zmin)->3, "zmin!")
		("zmax", po::value<std::float>()(&zmax)->6, "zmax!");

	// Backwards compatibility (no named parameters)
	po::positional_options_description positionalOptions;
	    positionalOptions.add("resolution", 1);
	    positionalOptions.add("zmin", 1);
	    positionalOptions.add("zmax", 1);
	    positionalOptions.add("downsample", 1);

    po::variables_map vm;

    po::store(po::command_line_parser(argc, argv).options(desc)
                      .positional(positionalOptions).run(),
                    vm);

    // throws an error if something is wrong:
    po::notify(vm);

	// float resolution = atof(argv[1]);
	// float zmin = atof(argv[2]);
	// float zmax = atof(argv[3]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plotcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::cout << "Reading plotcloud..." << std::endl;
	reader.read(downsample,*plotcloud);
	std::cout << "Finished reading plotcloud..." << std::endl;
	std::vector<std::string> id = getFileID(downsample);
	std::stringstream ss;
	ss.str("");
	ss << id[1] << ".slice.downsample.pcd";
	std::vector<std::vector<float>> dem;
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "Running getDemAndSlice..." << std::endl;
	dem = getDemAndSlice(plotcloud,resolution,zmin,zmax,slice);
	for(int j=0;j<dem.size();j++) std::cout << dem[j][0] << " " << dem[j][1] << " " << dem[j][2] << std::endl;
	std::cout << "Writing slice..." << std::endl;
	writer.write(ss.str(),*slice,true);
	return 0;
}
