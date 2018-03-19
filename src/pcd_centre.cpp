//Kim Calders: kim.calders@npl.co.uk

#include <treeseg.hpp>

int main (int argc, char** argv)
{
	float area = atof(argv[1]);
	float half_length = sqrt(area)/2;
	for(int i=2;i<argc;i++)
	{
		std::string fname;
		std::vector<std::string> name1;
		std::vector<std::string> name2;
//		std::vector<std::string> name3;
		boost::split(name1,argv[i],boost::is_any_of("."));
//		boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
//		boost::split(name3,name2[name2.size()-1],boost::is_any_of("_"));
//		fname = name3[name3.size()-1];
		boost::split(name2,name1[name1.size()-3],boost::is_any_of("_"));
		fname = name2[name2.size()-1];	
		pcl::PCDReader reader;
		pcl::PCDWriter writer;
		pcl::PointCloud<pcl::PointXYZ>::Ptr plot(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*plot);
		//
		Eigen::Vector4f min,max,centroid;
		pcl::compute3DCentroid(*plot,centroid);
		pcl::getMinMax3D(*plot,min,max);
//		std::cout << (max[0]-min[0])/2 << " " << (max[1]-min[1])/2 <<" " << std::endl;
//		std::cout << fname << " " << centroid[0] <<" " << centroid[1] << std::endl;
		std::cout << fname << " " << min[0]+half_length <<" " << min[1]+half_length << std::endl;
		//return 0;
	}
}
