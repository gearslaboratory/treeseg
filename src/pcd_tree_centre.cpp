//Kim Calders: kim.calders@npl.co.uk

#include <treeseg.hpp>

int main (int argc,char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	for (int i=1;i<argc;i++)
	{
		std::string fname;
		std::vector<std::string> name1;
		boost::split(name1,argv[i],boost::is_any_of("."));
		fname=name1[0];
		Eigen::Vector4f centroid;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*cloud);
		pcl::compute3DCentroid(*cloud,centroid);
		std::cout << fname << " " << centroid[0] <<" " << centroid[1] << std::endl;
	}
	return 0;
}
