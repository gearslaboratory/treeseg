//Andrew Burt - a.burt.12@ucl.ac.uk
// Modified by Kim Calders: read in multiple cluster files, merge them into 1 pcd, with each initial cluster having a different colour

#include <treeseg.hpp>



int main (int argc, char** argv)
{
	std::stringstream ss;
	ss << "merged_clusters.pcd";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCDReader reader;
	for(int i=1;i<argc;i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_in(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*cloud_tmp_in);

		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		for(int j=0;j<cloud_tmp_in->points.size();j++)
		{
			pcl::PointXYZRGB point;
			point.x = cloud_tmp_in->points[j].x;
			point.y = cloud_tmp_in->points[j].y;
			point.z = cloud_tmp_in->points[j].z;
			point.r = r;
			point.g = g;
			point.b = b;
			out_cloud->insert(out_cloud->end(),point);
		}
	}
	pcl::PCDWriter writer;
	writer.write(ss.str(),*out_cloud,true);
}



