//Andrew Burt - a.burt.12@ucl.ac.uk

#include <treeseg.hpp>

float nearestNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int NSearches)
{
	std::vector<float> dist;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	int K = NSearches + 1;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();it++)
	{
		std::vector<float> p_dist;
		pcl::PointXYZ searchPoint;
		searchPoint.x = it->x;
		searchPoint.y = it->y;
		searchPoint.z = it->z;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		tree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);
		for(int i=1;i<pointIdxNKNSearch.size();i++)
		{
			p_dist.push_back(sqrt(pointNKNSquaredDistance[i]));
		}
		float p_dist_sum = std::accumulate(p_dist.begin(),p_dist.end(),0.0);
		float p_dist_mean = p_dist_sum/p_dist.size();
		dist.push_back(p_dist_mean);
	}
	float dist_sum = std::accumulate(dist.begin(),dist.end(),0.0);
	float dist_mean = dist_sum/dist.size();
	return dist_mean;
}

struct cylParams
{
	bool ismodel;
	float x,y,z;
	float dx,dy,dz;
	float rad;
	float len;
	float ccount,icount;
	float steprad;
	float stepcov;
	float radratio;
	float ransac_dist;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

cylParams fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,int NSearch)
{
	cylParams cylinder;
	float dist = nearestNeighbour(cloud,NSearch);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
//	ne.setRadiusSearch(dist);
	ne.setKSearch(NSearch);
	ne.compute(*normals);
	cylParams tmp_cylinder;
	pcl::PointIndices indices;
	pcl::ModelCoefficients coeff;
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000000);
	seg.setDistanceThreshold(dist);
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.segment(indices,coeff);
	if(indices.indices.size() != 0)
	{
		//get x,y,z,dx,dy,dz,rad,len,indices
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_transformed(new pcl::PointCloud<pcl::PointXYZ>);		
		for(std::vector<int>::iterator pit=indices.indices.begin();pit!=indices.indices.end();pit++)
		{
			inliers->insert(inliers->end(),cloud->points[*pit]);
		}
		Eigen::Vector3f point(coeff.values[0],coeff.values[1],coeff.values[2]);
		Eigen::Vector3f direction(coeff.values[3],coeff.values[4],coeff.values[5]); 
		Eigen::Affine3f transform,inverse_transform;
		Eigen::Vector3f world(0,direction[2],-direction[1]); 
		direction.normalize(); 
		pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform); 
		pcl::transformPointCloud(*inliers,*inliers_transformed,transform); 
		Eigen::Vector4f min,max,centroid;
		pcl::getMinMax3D(*inliers_transformed,min,max);
		pcl::compute3DCentroid(*inliers_transformed,centroid);
		pcl::PointXYZ center_w,center;
		center_w.x = centroid[0];
		center_w.y = centroid[1];
		center_w.z = centroid[2];
		inverse_transform = transform.inverse();
		center = pcl::transformPoint(center_w,inverse_transform);
		cylinder.ismodel = true;
		cylinder.ismodel = true;
		cylinder.x = center.x;
		cylinder.y = center.y;
		cylinder.z = center.z;
		cylinder.dx = coeff.values[3];
		cylinder.dy = coeff.values[4];
		cylinder.dz = coeff.values[5];
		cylinder.rad = coeff.values[6];
		cylinder.len = max[2] - min[2];
		cylinder.ccount = cloud->points.size();
		cylinder.icount = inliers->points.size();
		cylinder.ransac_dist = dist;
		cylinder.cloud = inliers;
		//get radcov
		int RADCOVSTEPS = 6;
		float delta_z = (max[2]-min[2])/RADCOVSTEPS;
		std::vector<float> rad;
		for(int j=0;j<RADCOVSTEPS;j++)
		{
			float z_min = min[2] + j * delta_z;
			float z_max = min[2] + (j+1) * delta_z;
			pcl::PointCloud<pcl::PointXYZ>::Ptr z_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PassThrough<pcl::PointXYZ> passz;
			passz.setInputCloud(inliers_transformed);
			passz.setFilterFieldName("z");
			passz.setFilterLimits(z_min,z_max);
			passz.filter(*z_cloud);
			if(z_cloud->points.size() > 1)
			{
				pcl::PointCloud<pcl::Normal>::Ptr z_normals(new pcl::PointCloud<pcl::Normal>);
				pcl::search::KdTree<pcl::PointXYZ>::Ptr z_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
				pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> z_ne;
				z_ne.setSearchMethod(z_tree);
				z_ne.setInputCloud(z_cloud);
			//	z_ne.setRadiusSearch(dist);
				z_ne.setKSearch(NSearch);
				z_ne.compute(*z_normals);
				pcl::PointIndices z_indices;
				pcl::ModelCoefficients z_coeff;
				pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> z_seg;
				z_seg.setOptimizeCoefficients(true);
				z_seg.setModelType(pcl::SACMODEL_CYLINDER);
				z_seg.setNormalDistanceWeight(0.1);
				z_seg.setMethodType(pcl::SAC_RANSAC);
				z_seg.setMaxIterations(1000000);
				z_seg.setDistanceThreshold(dist);
				z_seg.setInputCloud(z_cloud);
				z_seg.setInputNormals(z_normals);
				z_seg.segment(z_indices,z_coeff);
				if(z_indices.indices.size() != 0)
				{
					if(z_coeff.values[6] > 0)
					{
						rad.push_back(z_coeff.values[6]);
					}
				}
			}
		}
		if(rad.size() >= RADCOVSTEPS-2) //i.e. wouldnt expect more than two steps to fail + needs to be bigger than 3
		{
			float sum = std::accumulate(rad.begin(),rad.end(),0.0);
			float mean = sum/rad.size();
			std::vector<float> diff(rad.size());
			std::transform(rad.begin(),rad.end(),diff.begin(),std::bind2nd(std::minus<float>(),mean));
			float stdev = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / rad.size());
			float cov = stdev / mean;
			cylinder.steprad = mean; 
			cylinder.stepcov = cov;
			cylinder.radratio = std::min(cylinder.rad,cylinder.steprad)/std::max(cylinder.rad,cylinder.steprad);
		}
		else cylinder.ismodel = false;
	}
	else cylinder.ismodel = false;
	return cylinder;
}

void segmentvolume(pcl::PointCloud<pcl::PointXYZ>::Ptr plot, cylParams cylinder, float radius_extension, pcl::PointCloud<pcl::PointXYZ>::Ptr &volume)
{
	float rad = cylinder.rad * radius_extension;
	float len = 1000;
	Eigen::Vector3f cp1(cylinder.x+len*cylinder.dx,cylinder.y+len*cylinder.dy,cylinder.z+len*cylinder.dz);
	Eigen::Vector3f cp2(cylinder.x-len*cylinder.dx,cylinder.y-len*cylinder.dy,cylinder.z-len*cylinder.dz);
	Eigen::Vector3f cn1 = cp2 - cp1;
	cn1.normalize(); 
	Eigen::Vector3f cn2 = -cn1;
	for(int i=0;i<plot->points.size();i++)
	{
		Eigen::Vector3f p(plot->points[i].x,plot->points[i].y,plot->points[i].z);
		float dot1 = cn1.dot(p-cp1);
		if(dot1 > 0)
		{
			float dot2 = cn2.dot(p-cp2);
			if(dot2 > 0)
			{
				Eigen::Vector3f mp = p - (cn1 * cn1.dot(p-cp1));
				float dist = sqrt(pow(mp(0)-cp1(0),2)+pow(mp(1)-cp1(1),2)+pow(mp(2)-cp1(2),2));
				if(dist <= rad)
				{
					volume->insert(volume->end(),plot->points[i]);
				}
			}
		}
	}
}

float zfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int min_count)
{
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	float delta_z = 0.5;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(float z=min[2];z<max[2];z+=delta_z)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr step_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z,z+delta_z);
		pass.filter(*step_cloud);
		if(step_cloud->points.size() > min_count) *filtered_cloud += *step_cloud;
	}
	*cloud = *filtered_cloud;
}

void segmentground(pcl::PointCloud<pcl::PointXYZ>::Ptr &volume, float NSearch, float delta_z)
{
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*volume,min,max);
	pcl::PointCloud<pcl::PointXYZ>::Ptr top(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr bottom(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(volume);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min[2],min[2]+delta_z);
	pass.filter(*bottom);
	pass.setFilterLimitsNegative(true);
	pass.filter(*top);
	float dist = nearestNeighbour(bottom,NSearch);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(dist);
	seg.setInputCloud(bottom);
	seg.segment(*inliers,*coefficients);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(bottom);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*bottom);
	*volume = *top + *bottom;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &volume_cloud, int NSearch, int min_count, float delta_z)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*volume_cloud,min,max);
	for(float z=min[2];z<max[2];z+=delta_z)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr z_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> passz;
		passz.setInputCloud(volume_cloud);
		passz.setFilterFieldName("z");
		passz.setFilterLimits(z,z+delta_z);
		passz.filter(*z_cloud);	
		if(z_cloud->points.size() != 0)
		{
			float dist = nearestNeighbour(z_cloud,NSearch);
			std::vector<pcl::PointIndices> indices;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance(dist);
			ec.setMinClusterSize(min_count);
			ec.setMaxClusterSize(std::numeric_limits<int>().max());
			ec.setSearchMethod(tree);
			ec.setInputCloud(z_cloud);
			ec.extract(indices);
			for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
				{
					cloud->insert(cloud->end(),z_cloud->points[*pit]);
				}
				clusters.push_back(cloud);
			}
		}
	}
	return clusters;
}

void clusterCorrection(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, int NSearch, int min_count, float smoothness)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> corrected_clusters;
	for(int i=0;i<clusters.size();i++)
	{
//		float dist = nearestNeighbour(clusters[i],NSearch);
		std::vector<pcl::PointIndices> indices;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
		ne.setSearchMethod(tree);
		ne.setInputCloud(clusters[i]);
//		ne.setRadiusSearch(dist);
		ne.setKSearch(NSearch);
		ne.compute(*normals);
		pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> reg;
		reg.setMinClusterSize(min_count);
		reg.setMaxClusterSize(1000000);
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30); //this is a sensitive value - keep fixed and vary smoothness and curvature
		reg.setInputCloud(clusters[i]);
		reg.setInputNormals(normals);
		reg.setSmoothnessThreshold(smoothness/ 180 * M_PI);
		reg.extract(indices);
		for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
			{
				cloud->insert(cloud->end(),clusters[i]->points[*pit]);
			}
			corrected_clusters.push_back(cloud);
		}
	}
	clusters.clear();
	clusters = corrected_clusters;
}

float getdistance(pcl::PointCloud<pcl::PointXYZ>::Ptr a, pcl::PointCloud<pcl::PointXYZ>::Ptr b)
{
	float distance = std::numeric_limits<float>::max();
	for(int i=0;i<a->points.size();i++)
	{
		for(int j=0;j<b->points.size();j++)
		{
			float d = pcl::euclideanDistance(a->points[i],b->points[j]);
			if(d < distance) distance = d;
		}
	}
	return distance;
}

void buildstem(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, float distance_max, float angle_max, float eigenratio_max, float delta_z, pcl::PointCloud<pcl::PointXYZ>::Ptr &stem)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cc(new pcl::PointCloud<pcl::PointXYZ>);
	for(int p=0;p<clusters.size();p++) *cc += *clusters[p];
	Eigen::Vector4f cmin,cmax;
	pcl::getMinMax3D(*cc,cmin,cmax);
	//get most conformant constituant to cluster
	Eigen::Vector4f c_min,c_max;
	Eigen::Vector4f c_centroid;
	Eigen::Matrix3f c_covariancematrix;
	Eigen::Matrix3f c_eigenvectors;
	Eigen::Vector3f c_eigenvalues;
	pcl::getMinMax3D(*cluster,c_min,c_max);
	pcl::compute3DCentroid(*cluster,c_centroid);
	pcl::computeCovarianceMatrix(*cluster,c_centroid,c_covariancematrix);
	pcl::eigen33(c_covariancematrix,c_eigenvectors,c_eigenvalues);
	Eigen::Vector4f c_vector(c_eigenvectors(0,2),c_eigenvectors(1,2),c_eigenvectors(2,2),0);
	std::vector<float> metrics;
	for(int i=0;i<clusters.size();i++)
	{
		Eigen::Vector4f min,max;
		Eigen::Vector4f centroid;
		Eigen::Matrix3f covariancematrix;
		Eigen::Matrix3f eigenvectors;
		Eigen::Vector3f eigenvalues;
		pcl::getMinMax3D(*clusters[i],min,max);
		pcl::compute3DCentroid(*clusters[i],centroid);
		pcl::computeCovarianceMatrix(*clusters[i],centroid,covariancematrix);
		pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
		Eigen::Vector4f vector(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2),0);
//		if(centroid[2] > c_centroid[2])
//		{
			float distance = getdistance(cluster,clusters[i]);
			//
			float angle = pcl::getAngle3D(c_vector,vector) * (180/M_PI);
			if(angle > 90) angle = 180 - angle;
			//
			if(eigenvectors(2) > c_eigenvectors(2)) eigenvectors(2) = c_eigenvectors(2);
			float eigenratio = std::max(c_eigenvalues(2),eigenvalues(2)) / std::min(c_eigenvalues(2),eigenvalues(2));
			//
			float distance_normalised = (distance - 0) / (distance_max - 0);
			float angle_normalised = (angle - 0) / (angle_max - 0);
			float eigenratio_normalised = (eigenratio - 1) / (eigenratio_max - 1);
			float metric = (distance_normalised + eigenratio_normalised) / 3;
			metrics.push_back(metric);
//		}
//		else metrics.push_back(1000);
	}
	int idx = std::distance(metrics.begin(),std::min_element(metrics.begin(),metrics.end()));
	pcl::PointCloud<pcl::PointXYZ>::Ptr outer(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> trunk;
	*outer = *clusters[idx];
	*tmp = *clusters[idx];
	trunk.push_back(clusters[idx]);
	clusters.erase(clusters.begin()+idx);
	//work down
	bool done_something = true;
	while(done_something == true)
	{
		if(clusters.size() != 0)
		{
			Eigen::Vector4f c_min,c_max;
			Eigen::Vector4f c_centroid;
			Eigen::Matrix3f c_covariancematrix;
			Eigen::Matrix3f c_eigenvectors;
			Eigen::Vector3f c_eigenvalues;
			pcl::getMinMax3D(*outer,c_min,c_max);
			pcl::compute3DCentroid(*outer,c_centroid);
			pcl::computeCovarianceMatrix(*outer,c_centroid,c_covariancematrix);
			pcl::eigen33(c_covariancematrix,c_eigenvectors,c_eigenvalues);
			Eigen::Vector4f c_vector(c_eigenvectors(0,2),c_eigenvectors(1,2),c_eigenvectors(2,2),0);
			std::vector<float> metrics;
			for(int j=0;j<clusters.size();j++)
			{
				Eigen::Vector4f min,max;
				Eigen::Vector4f centroid;
				Eigen::Matrix3f covariancematrix;
				Eigen::Matrix3f eigenvectors;
				Eigen::Vector3f eigenvalues;
				pcl::getMinMax3D(*clusters[j],min,max);
				pcl::compute3DCentroid(*clusters[j],centroid);
				pcl::computeCovarianceMatrix(*clusters[j],centroid,covariancematrix);
				pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
				Eigen::Vector4f vector(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2),0);
				if(centroid[2] < c_centroid[2])
				{
					//this is a special case
					if(eigenvalues(2) > c_eigenvalues(2)) eigenvalues(2) = c_eigenvalues(2);
					float distance = getdistance(outer,clusters[j]);
					//
					float angle = pcl::getAngle3D(c_vector,vector) * (180/M_PI);
					if(angle > 90) angle = 180 - angle;
					float eigenratio = std::max(c_eigenvalues(2),eigenvalues(2)) / std::min(c_eigenvalues(2),eigenvalues(2));					//
					float distance_normalised = (distance - 0) / (distance_max - 0);
					float angle_normalised = (angle - 0) / (angle_max - 0);
					float eigenratio_normalised = (eigenratio - 1) / (eigenratio_max - 1);
					float metric = (distance_normalised + eigenratio_normalised + angle_normalised) / 3;
					metrics.push_back(metric);
				}
				else metrics.push_back(1000);
			}
			int idx = std::distance(metrics.begin(),std::min_element(metrics.begin(),metrics.end()));
			if(metrics[idx] < 1)
			{
				trunk.push_back(clusters[idx]);
				*outer = *clusters[idx];
				clusters.erase(clusters.begin()+idx);
			}
			else done_something = false;
		}
		else done_something = false;
	}
	//work upawards
	*outer = *tmp;
	done_something = true;
	while(done_something == true)
	{
		if(clusters.size() != 0)
		{
			Eigen::Vector4f c_min,c_max;
			Eigen::Vector4f c_centroid;
			Eigen::Matrix3f c_covariancematrix;
			Eigen::Matrix3f c_eigenvectors;
			Eigen::Vector3f c_eigenvalues;
			pcl::getMinMax3D(*outer,c_min,c_max);
			pcl::compute3DCentroid(*outer,c_centroid);
			pcl::computeCovarianceMatrix(*outer,c_centroid,c_covariancematrix);
			pcl::eigen33(c_covariancematrix,c_eigenvectors,c_eigenvalues);
			Eigen::Vector4f c_vector(c_eigenvectors(0,2),c_eigenvectors(1,2),c_eigenvectors(2,2),0);
			std::vector<float> metrics;
			for(int j=0;j<clusters.size();j++)
			{
				Eigen::Vector4f min,max;
				Eigen::Vector4f centroid;
				Eigen::Matrix3f covariancematrix;
				Eigen::Matrix3f eigenvectors;
				Eigen::Vector3f eigenvalues;
				pcl::getMinMax3D(*clusters[j],min,max);
				pcl::compute3DCentroid(*clusters[j],centroid);
				pcl::computeCovarianceMatrix(*clusters[j],centroid,covariancematrix);
				pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
				Eigen::Vector4f vector(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2),0);
				if(centroid[2] > c_centroid[2])
				{
					float distance = getdistance(outer,clusters[j]);
					//
					float angle = pcl::getAngle3D(c_vector,vector) * (180/M_PI);
					if(angle > 90) angle = 180 - angle;
					float eigenratio = std::max(c_eigenvalues(2),eigenvalues(2)) / std::min(c_eigenvalues(2),eigenvalues(2));
					//
					float distance_normalised = (distance - 0) / (distance_max - 0);
					float angle_normalised = (angle - 0) / (angle_max - 0);
					float eigenratio_normalised = (eigenratio - 1) / (eigenratio_max - 1);
					float metric = (distance_normalised + eigenratio_normalised + angle_normalised) / 3;
					metrics.push_back(metric);
				}
				else metrics.push_back(1000);
			}
			int idx = std::distance(metrics.begin(),std::min_element(metrics.begin(),metrics.end()));
			if(metrics[idx] < 1)
			{
				trunk.push_back(clusters[idx]);
				*outer = *clusters[idx];
				clusters.erase(clusters.begin()+idx);
			}
			else done_something = false;
		}
		else done_something = false;
	}
	for(int k=0;k<trunk.size();k++) *stem += *trunk[k];
}

void segmentlocal(pcl::PointCloud<pcl::PointXYZ>::Ptr &stem, float NSearch, float delta_z, float radius_extension)
{
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*stem,min,max);
	pcl::PointCloud<pcl::PointXYZ>::Ptr stem_new(new pcl::PointCloud<pcl::PointXYZ>);
	for(float z=min[2];z<max[2];z+=delta_z)
	{	
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr z_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(stem);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z,z+delta_z);
		pass.filter(*z_cloud);
		cylParams cyl = fitCylinder(z_cloud,NSearch);
		segmentvolume(z_cloud,cyl,radius_extension,tmp);
		*stem_new += *tmp;
	}
	*stem = *stem_new;
}

void stemcorrection(pcl::PointCloud<pcl::PointXYZ>::Ptr &stem, float NSearch, float delta_z, float start, float stepcov_max, float radchange_min)
{
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*stem,min,max);
	float z_max = max[2];
	float rad_previous;
	pcl::PassThrough<pcl::PointXYZ> pass;
	for(float z=min[2]+start;z<max[2];z+=delta_z)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr z_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pass.setInputCloud(stem);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z,z+delta_z);
		pass.filter(*z_cloud);
		float dist = nearestNeighbour(z_cloud,NSearch);
		pcl::PointIndices inliers; 
		pcl::ModelCoefficients coefficients; 
		pcl::SACSegmentation<pcl::PointXYZ> seg; 
		seg.setOptimizeCoefficients(true); 
		seg.setMethodType(pcl::SAC_RANSAC); 
		seg.setMaxIterations(10000); 
		seg.setModelType(pcl::SACMODEL_CIRCLE2D); 
		seg.setDistanceThreshold(dist); 
		seg.setInputCloud(z_cloud);
		seg.segment(inliers,coefficients);
		//
		float steps = 5;
		Eigen::Vector4f zmin,zmax;
		pcl::getMinMax3D(*z_cloud,zmin,zmax);
		float dz = (zmax[2] - zmin[2]) / steps;
		int count = 0;
		float zz = zmin[2];
		std::vector<float> rad;
		while(count < steps)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr zz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pass.setInputCloud(stem);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(zz,zz+dz);
			pass.filter(*zz_cloud);
			if(zz_cloud->points.size() > 3)
			{
				pcl::PointIndices zz_inliers; 
				pcl::ModelCoefficients zz_coefficients; 
				pcl::SACSegmentation<pcl::PointXYZ> seg; 
				seg.setOptimizeCoefficients(true); 
				seg.setMethodType(pcl::SAC_RANSAC); 
				seg.setMaxIterations(10000); 
				seg.setModelType(pcl::SACMODEL_CIRCLE2D); 
				seg.setDistanceThreshold(dist); 
				seg.setInputCloud(zz_cloud);
				seg.segment(zz_inliers,zz_coefficients);
				rad.push_back(zz_coefficients.values[2]);
			}
			zz += dz;
			count++;
		}
		if(rad.size() > 3)
		{
			float sum = std::accumulate(rad.begin(),rad.end(),0.0);
			float mean = sum/rad.size();
			std::vector<float> diff(rad.size());
			std::transform(rad.begin(),rad.end(),diff.begin(),std::bind2nd(std::minus<float>(),mean));
			float stdev = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / rad.size());
			float cov = stdev / mean;
			float radchange = std::min(coefficients.values[2],mean) / std::max(coefficients.values[2],mean);
			std::cout << coefficients.values[2] << " " << mean << " " << std::min(coefficients.values[2],mean)/std::max(coefficients.values[2],mean) << " " << cov << std::endl;
			if(cov > stepcov_max)
			{
				z_max = z-delta_z;
				break;
			}	
			if(radchange < radchange_min)
			{
				z_max = z-delta_z;
				break;
			}
		}
		else
		{
			z_max = z-delta_z;
			break;
		}
	}
	pass.setInputCloud(stem);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min[2],z_max);
	pass.filter(*stem);
}

void writeClusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::string fname, bool individual)
{
	pcl::PCDWriter writer;
	if(individual == false)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(int i=0;i<clusters.size();i++)
		{
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			for(int j=0;j<clusters[i]->points.size();j++)
			{
				pcl::PointXYZRGB point;
				point.x = clusters[i]->points[j].x;
				point.y = clusters[i]->points[j].y;
				point.z = clusters[i]->points[j].z;
				point.r = r;
				point.g = g;
				point.b = b;
				out_cloud->insert(out_cloud->end(),point);
			}
		}
		writer.write(fname,*out_cloud,true);
	}
	if(individual == true)
	{
		int count = 0;
		for(int i=0;i<clusters.size();i++)
		{
			std::stringstream ss;
			ss << "tree_" << count << ".pcd";
			writer.write(ss.str(),*clusters[i],true);
			count++;
		}
	}
}

int main (int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr plot(new pcl::PointCloud<pcl::PointXYZ>);
//	int ntiles= atoi(argv[2]) + 3;
//	for(int i=3;i<ntiles;i++)
//	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
//		reader.read(argv[i],*cloud_tmp);
//		std::cout << argv[i] << std::endl;
//		*plot += *cloud_tmp;
//	}
//	reader.read(argv[1],*plot);
	for(int i=6;i<argc;i++)
	{
		std::cout << "---------------" << std::endl;
		//
		std::string fname;
		std::vector<std::string> name1;
		std::vector<std::string> name2;
		std::vector<std::string> name3;
		boost::split(name1,argv[i],boost::is_any_of("."));
		boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
		boost::split(name3,name2[name2.size()-1],boost::is_any_of("_"));
		fname = name3[name3.size()-1];
		std::stringstream ss;
		ss << "stem_" << fname << ".pcd";
		std::string ll;
		ll += fname;
		ll += ".tracker";
		std::cout << argv[i] << " " << ss.str() << std::endl;
		if ( !boost::filesystem::exists(ss.str()) )
		{
			if ( !boost::filesystem::exists(ll) )
			{		
				//
				std::ofstream o(ll.c_str()); //creates empty file; just here so loop can check if this tree is being processed already
				pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
				reader.read(argv[i],*cluster);
				//
				//find centre of cluster
				Eigen::Vector4f min,max,centroid;
				pcl::compute3DCentroid(*cluster,centroid);
				std::ifstream infile(argv[4]);
				int tile;
				float x,y;
				std::list<int> tiles_list;
				while (infile >> tile >> x >> y)
				{
					float expansion = atof(argv[5]);
					if(expansion < 11.0)
					{
						std::cout << "Updating search radius to 11m" << std::endl;						
						expansion = 11.0;
					}
					float xmin = centroid[0]-expansion; 
					float xmax = centroid[0]+expansion;
					float ymin = centroid[1]-expansion;
					float ymax= centroid[1]+expansion;
					if(x >= xmin && x <= xmax && y >= ymin && y <= ymax)
					{
		//				std::cout << tile << " " << x <<" " << y << std::endl;
						tiles_list.push_back(tile);
					}
				}	
				pcl::PointCloud<pcl::PointXYZ>::Ptr plot(new pcl::PointCloud<pcl::PointXYZ>);		
				std::list<int>::iterator it;
				for (it = tiles_list.begin(); it != tiles_list.end(); ++it){
		//    			std::cout << *it << std::endl;
					std::stringstream ss;
					ss.str("");
					ss << argv[2] << argv[3] << "_" << *it << ".downsample.pcd";
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);		
					reader.read(ss.str(),*cloud_tmp);
		    			std::cout << ss.str() << std::endl;
					*plot += *cloud_tmp;		
				}
				//
				std::cout << "RANSAC Cylinder fit: " << std::flush;
				int NSearch = 60; //10x voxel neighbours
				cylParams cylinder = fitCylinder(cluster,NSearch);
				std::cout << cylinder.rad << std::endl;
				if(cylinder.rad > 0 && cylinder.rad < 3.0) //added max rad; otherwise a massive cloud will be extracted and you might run out of memory. Need to check such clusters
				{
				//
				std::cout << "Segmenting extended cylinder: " << std::flush;
				pcl::PointCloud<pcl::PointXYZ>::Ptr volume(new pcl::PointCloud<pcl::PointXYZ>);
				float radius_extension = 10; //6; //butresses, dvector changes etc
				segmentvolume(plot,cylinder,radius_extension,volume);
				zfilter(volume,10);
		//debug		reader.read(argv[3],*volume);
				ss.str("");
				ss << "cylinder_" << fname << ".pcd";
				writer.write(ss.str(),*volume,true);
				std::cout << ss.str() << std::endl;
				//
				std::cout << "Segmenting ground plane: " << std::flush;
				NSearch = 60; //10x voxel neighbours
				float delta_z = 0.75;
				segmentground(volume,NSearch,delta_z);
				ss.str("");
				ss << "cylinder_noground_" << fname << ".pcd";
				writer.write(ss.str(),*volume,true);
				std::cout << ss.str() << std::endl;
				//
				std::cout << "Cluster extraction: " << std::flush;
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
				NSearch = 9; //1.5x voxel neighbours
				int min_count = 10;
				delta_z = 3; //getslice!
				clusters = clusterSegmentation(volume,NSearch,min_count,delta_z);
				ss.str("");
				ss << "cylinder_noground_clusters_" << fname << ".pcd"; 
				writeClusters(clusters,ss.str(),false);
				std::cout << ss.str() << std::endl;
				//
				std::cout << "Cluster correction: " << std::flush;
				NSearch = 9; //1.5x voxel neighbours;
				float smoothness = atof(argv[1]);
				clusterCorrection(clusters,NSearch,min_count,smoothness);
				ss.str("");
				ss << "cylinder_noground_clusters_corrected_" << fname << ".pcd";
				writeClusters(clusters,ss.str(),false);
				std::cout << ss.str() << std::endl;
				//
				std::cout << "Building stem: " << std::flush;
				pcl::PointCloud<pcl::PointXYZ>::Ptr stem(new pcl::PointCloud<pcl::PointXYZ>);
				float distance_max = 0.05; //voxel downsample
				float angle_max = 30;
				float eigenratio_max = 50;
				buildstem(cluster,clusters,distance_max,angle_max,eigenratio_max,delta_z,stem);
				ss.str("");
				ss << "cylinder_noground_clusters_corrected_stem_" << fname << ".pcd";
				writer.write(ss.str(),*stem,true);
				std::cout << ss.str() << std::endl;
				//
		//		std::cout << "Localised cylinder fit: " << std::flush;
		//		NSearch = 60; //10x voxel neighbours
		//		delta_z = 1.5;
		//		radius_extension = 2;
		//		segmentlocal(stem,NSearch,delta_z,radius_extension);
		//		ss.str("");
		//		ss << "cylinder_noground_clusters_corrected_stem_local_" << fname << ".pcd";
		//		writer.write(ss.str(),*stem,true);
		//		std::cout << ss.str() << std::endl;
				//
				std::cout << "Stem correction: " << std::endl;
				NSearch = 60;
				delta_z = 0.75;
				float start = 5;
				float stepcov_max = 0.05;
				float radchange_min = 0.9;
				stemcorrection(stem,NSearch,delta_z,start,stepcov_max,radchange_min);
				ss.str("");
				ss << "stem_" << fname << ".pcd";
				writer.write(ss.str(),*stem,true);
				std::cout << ss.str() << std::endl;
				//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				//some very weird intermittent shit going on here
				//temp fix:
				cylinder.rad = 0;
				//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				}
				else std::cout << "MISSED HERE" << std::endl;
			}
		}
	}
	return 0;
}
