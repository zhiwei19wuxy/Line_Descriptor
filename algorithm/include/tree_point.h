/**
 *  \file   tree_point.h
 *  \brief
 *  \author wxy
 *  \email  wuxiaoyang22@mails.ucas.ac.cn
 *  \date   2023/12/21
 *  \note   
 */
//

#pragma once
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;

class PointExtraction
{
 public:
//  cloudPtr inputCloud;
//  cloudPtr seedCloud;
//  int k;//K邻域参数
	PointExtraction(){
		seed_cloud_temp = cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
}
	void Extract_Point(cloudPtr cloud, cloudPtr cloud_treePoint, int k);
	void extract_fine(cloudPtr input_cloud, cloudPtr temp_cloud, int k,cloudPtr seed_cloud_final);

	void calculate_height(cloudPtr temp_cloud,cloudPtr input_cloud,float grid_size);

 private:
	cloudPtr seed_cloud_temp;


};


