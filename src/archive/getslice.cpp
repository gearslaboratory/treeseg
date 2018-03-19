//Andrew Burt - a.burt.12@ucl.ac.uk

#include <treeseg.hpp>

float zfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	float delta_z = 1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(float z=min[2];z<max[2];z+=delta_z)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr step_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z,z+delta_z);
		pass.filter(*step_cloud);
		if(step_cloud->points.size() > 5) *filtered_cloud += *step_cloud;
	}
	*cloud = *filtered_cloud;
}

int main (int argc, char** argv)
{
	float resolution = atof(argv[1]);
	float z_min = atof(argv[2]);
	float z_max = atof(argv[3]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	reader.read(argv[4],*cloud_in);
	std::string fname;
	std::vector<std::string> name1;
	std::vector<std::string> name2;
	std::vector<std::string> name3;
	boost::split(name1,argv[4],boost::is_any_of("."));
	fname = name1[name1.size()-3];
	std::stringstream ss;
	ss << fname << ".slice.downsample.pcd";
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud_in,min,max);
//	std::ofstream outfile("dem.txt");
//	outfile << resolution << " " << min[0] << " " << max[0] << " " << min[1] << " " << max[1] << std::endl;
	for(float x=min[0];x<=max[0];x+=resolution)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> passx;
		passx.setInputCloud(cloud_in);
		passx.setFilterFieldName("x");
		passx.setFilterLimits(x,x+resolution);
		passx.filter(*cloud_x);
		for(float y=min[1];y<=max[1];y+=resolution)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PassThrough<pcl::PointXYZ> passy;
			passy.setInputCloud(cloud_x);
			passy.setFilterFieldName("y");
			passy.setFilterLimits(y,y+resolution);
			passy.filter(*cloud_y);
			//
			zfilter(cloud_y);
			//
			Eigen::Vector4f min_s,max_s;
			pcl::getMinMax3D(*cloud_y,min_s,max_s);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PassThrough<pcl::PointXYZ> passz;
			passz.setInputCloud(cloud_y);
			passz.setFilterFieldName("z");
			passz.setFilterLimits(min_s[2]+z_min,min_s[2]+z_max);
			passz.filter(*cloud_z);
//			outfile << x << " " << y << " " << min_s[2] << std::endl;
			*cloud_out += *cloud_z;
		}
	}
//	outfile.close();
	writer.write(ss.str(),*cloud_out,true);
	return 0;
}
