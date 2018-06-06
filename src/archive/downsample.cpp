//Andrew Burt - a.burt.12@ucl.ac.uk

#include <treeseg.hpp>

int main (int argc,char** argv)
{
	int splits = atoi(argv[1]);
	float tolerance = atof(argv[2]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string fname;
	std::vector<std::string> name1;
	std::vector<std::string> name2;
	std::vector<std::string> name3;
	boost::split(name1,argv[3],boost::is_any_of("."));
	boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
	boost::split(name3,name2[name2.size()-1],boost::is_any_of("_"));
	fname = name3[name3.size()-2];
	std::stringstream ss;
	ss << fname << ".downsample.pcd";
	for(int i=3;i<argc;i++)
	{
		pcl::PCDReader reader;
		pcl::VoxelGrid<pcl::PointXYZ> downsample;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*cloud_in);
		if(splits != 0)
		{
			Eigen::Vector4f min,max;
			pcl::getMinMax3D(*cloud_in,min,max);
			float step = (max[0]-min[0])/splits;
			int count = 1;
			float xmin = min[0];
			float xmax = min[0]+step;
			while(count <= splits)
			{
//				std::cout << count << " " << xmin << " "  << xmax << std::endl;
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_pass(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_down(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PassThrough<pcl::PointXYZ> passx;
				passx.setInputCloud(cloud_in);
				passx.setFilterFieldName("x");
				passx.setFilterLimits(xmin,xmax);
				passx.filter(*cloud_tmp_pass);
				downsample.setInputCloud(cloud_tmp_pass);
				downsample.setLeafSize(tolerance,tolerance,tolerance);
				downsample.filter(*cloud_tmp_down);
				*cloud_down += * cloud_tmp_down;
				xmin += step;
				xmax += step;
				count++;
			}
		}
		else
		{	
			downsample.setInputCloud(cloud_in);
			downsample.setLeafSize(tolerance,tolerance,tolerance);
			downsample.filter(*cloud_down);
		}
		*downsample_cloud += *cloud_down;
	}
	pcl::PCDWriter writer;
	writer.write(ss.str(),*downsample_cloud,true);
	return 0;
}
