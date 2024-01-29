#include"tree_point.h"
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include"TINConstruct.h"
#include <pcl/io/pcd_io.h>

void PointExtraction::Extract_Point(cloudPtr cloud, cloudPtr cloud_treePoint, int k)
{
	// 创建KD树用于最近邻搜索
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	for (int i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ search_point = cloud->points[i];

		// 进行最近邻搜索
		std::vector<int> point_indices(k);
		std::vector<float> point_distances(k);
		kdtree.nearestKSearch(search_point, k, point_indices, point_distances);
		//kdtree.radiusSearch(search_point, r, point_indices, point_distances);

		// 获取当前点的高程值
		float current_elevation = search_point.z;
		bool is_seed_point = true;

		// 检查当前点的高程是否大于所有邻域点的高程
		for (int j = 0; j < k; ++j)
		{
			float neighbor_elevation;
			neighbor_elevation = cloud->points[point_indices[j]].z;

			// 如果当前点的高程小于任何一个邻域点的高程，则不将其保存为种子点
			if (current_elevation < neighbor_elevation)
			{
				is_seed_point = false;
				break; // 停止比较，因为已经找到一个邻域点高程大于等于的点
			}
		}

		// 如果当前点是种子点，则将其保存到种子点云中
		if (is_seed_point)
		{
			cloud_treePoint->points.push_back(search_point);
		}
	}
	cloud_treePoint->width = cloud_treePoint->size();
	cloud_treePoint->height = 1;
}
///
/// \param input_cloud
/// \param temp_cloud
/// \param k
/// \param seed_cloud

void PointExtraction::extract_fine(cloudPtr input_cloud, cloudPtr temp_cloud, int k,cloudPtr seed_cloud_final)
{
	float grid_size=2;
	calculate_height(temp_cloud,input_cloud,grid_size);
//	pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\cloud_angle.pcd", *seed_cloud_temp);

	bool is_stop = false;
	cloudPtr cloud_search(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_search = seed_cloud_temp;
	while (!is_stop)
	{
		int num_search = cloud_search->size();
		cloudPtr cloud_treePoint_temp(new pcl::PointCloud<pcl::PointXYZ>);
		// 创建KD树用于最近邻搜索
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(cloud_search);

		for (int i = 0; i < cloud_search->points.size(); ++i)
		{
			pcl::PointXYZ search_point = cloud_search->points[i];

			// 进行最近邻搜索
			std::vector<int> point_indices(k);
			std::vector<float> point_distances(k);
			kdtree.nearestKSearch(search_point, k, point_indices, point_distances);
			//kdtree.radiusSearch(search_point, r, point_indices, point_distances);

			bool is_seed_point = true;

			Eigen::Vector3f vector_z(0.0, 0.0, 1.0);
			//获取角度
			for (int j = 1; j < k; ++j)
			{
				pcl::PointXYZ point_neighbor(cloud_search->points[point_indices[j]]);//邻域点
				//获取两点之间的向量,2-1,是从1指到2
				Eigen::Vector3f vector(point_neighbor.x - search_point.x, point_neighbor.y - search_point.y, point_neighbor.z - search_point.z);
				//角度计算
				float angle = pcl::getAngle3D(vector, vector_z, true);
				float threshold_angle = 45;
				if (angle < threshold_angle)
				{
					is_seed_point = false;
					break;
				}
			}

			// 如果当前点是种子点，则将其保存到种子点云中
			if (is_seed_point)
			{
				cloud_treePoint_temp->points.push_back(search_point);
			}
		}
		cloud_search = cloud_treePoint_temp;
		if (cloud_treePoint_temp->size() == num_search)
		{
			is_stop = true;
		}
	}
	for (auto& pt : cloud_search->points)
	{
		seed_cloud_final->push_back(pt);
	}
	seed_cloud_final->width = seed_cloud_final->size();
	seed_cloud_final->height = 1;
	seed_cloud_temp->clear();
}

void PointExtraction::calculate_height(cloudPtr temp_cloud,cloudPtr input_cloud,float grid_size)
{
	//划分格网，根据格网最低点构建TIN，将point点落在三角面片上获取地面z值，差值即为树高
	//格网最低点：为避免格网最低点为非地面点造成的误差，将格网最低点与其邻域格网比较，差值大于阈值的剔除

	//划分格网
	pcl::PointXYZ point_min, point_max;
	pcl::getMinMax3D(*input_cloud, point_min, point_max);
	//获取最大值和最小值
	cloudPtr cloud_minmax(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_minmax->push_back(point_min);
	cloud_minmax->push_back(point_max);
	//pcl::io::savePCDFile<pcl::PointXYZ>("D:\\data\\registration\\TLS_1_minmax.pcd", *cloud_minmax);
	//如果min是最小的，可能会出现，相减为零的情况，因此，将其减小一点
//	point_min.x -= 0.00001;
//	point_min.y -= 0.00001;
	int rows = std::ceil((point_max.x - point_min.x) / grid_size);//计算行列数
	int cols = std::ceil((point_max.y - point_min.y) / grid_size);
	std::vector<std::vector<int>> grid_low;//最低点的索引
	std::vector<std::vector<std::vector<int>>> grid;//三维索引，存储全部的点
	grid_low.resize(rows);//初始化空间
	for (auto& row_low : grid_low)
	{
		row_low.resize(cols, -1);//初始化为-1

	}
	grid.resize(rows);//初始化grid的行
	for (auto& row : grid)
	{
		row.resize(cols);//列

	}
	for (int i = 0; i < input_cloud->size(); i++)
	{
		int row = std::ceil((input_cloud->points[i].x - point_min.x) / grid_size) - 1;
		int col = std::ceil((input_cloud->points[i].y - point_min.y) / grid_size) - 1;
		if(row==-1)
		{
			row=0;
		}
		if(col==-1)
		{
			col=0;
		}
		grid[row][col].push_back(i);
		if (grid_low[row][col] == -1)//如果是-1的话，将每个索引初始值认为是第一个
		{
			grid_low[row][col] = i;
		}
		else if (input_cloud->points[i].z < input_cloud->points[grid_low[row][col]].z)
		{
			grid_low[row][col] = i;//如果某个点比第一个点的z值还低，则将索引存到grid_low里面
		}
	}

	cloudPtr cloud_z0(new pcl::PointCloud<pcl::PointXYZ>);//z为0的格网最低点，为了进行k近邻索引，于是将z值赋值为0
	std::vector<std::pair<int, int>> idx_ij;
	for (int row = 0; row < grid_low.size(); row++)
	{
		for (int col = 0; col < grid_low[row].size(); col++)
		{
			if (grid_low[row][col] == -1)
			{
				continue;
			}
			cloud_z0->push_back(pcl::PointXYZ(
				input_cloud->points[grid_low[row][col]].x, input_cloud->points[grid_low[row][col]].y, 0));
			idx_ij.push_back(std::pair<int, int>(row, col));
			//height_grid.push_back(input_cloud->points[grid_low[i][j]].z);
		}
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_z0);
	float threshold = 1.0;
	std::vector<std::pair<int, int>> idx_removed;// 待移除的网格最低点索引
	std::vector<int> point_indices;
	std::vector<float> point_distances;
	for (int i = 0; i < cloud_z0->size(); i++)
	{
//		kdtree.radiusSearch(cloud_z0->points[i],2*grid_size, point_indices, point_distances);
		kdtree.nearestKSearch(cloud_z0->points[i], 15, point_indices, point_distances);

		for (int j = 0; j < point_indices.size(); j++)
		{
			float delta_height = input_cloud->points[grid_low[idx_ij[i].first][idx_ij[i].second]].z
				- input_cloud->points[grid_low[idx_ij[point_indices[j]].first][idx_ij[point_indices[j]].second]].z;
			if (delta_height > threshold)
			{
				idx_removed.push_back(idx_ij[i]);//将他存储起来，方便后续移除
				break;
			}
		}
	}

	for (const auto& idx_removed_item : idx_removed)
	{
		grid_low[idx_removed_item.first][idx_removed_item.second] = -1;//将他标志起来，方便后续移除
	}

	//格网最低点，加四个角点
	cloudPtr cloud_lowest(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < grid_low.size(); i++)
	{
		for (int j = 0; j < grid_low[i].size(); j++)
		{
			if (grid_low[i][j] == -1)
			{
				continue;
			}
			cloud_lowest->push_back(input_cloud->points[grid_low[i][j]]);
		}
	}
//	pcl::io::savePCDFile<pcl::PointXYZ>("D:\\data\\registration\\TLS_1_grid_lowest.pcd", *cloud_lowest);

	pcl::PointXYZ p1,p2,p3,p4;//p1左上角点，p2左下角点，p3右上角点，p4右下角点
	p1.x=point_min.x;
	p1.y=point_max.y;
	p1.z=0;
	p2.x=point_min.x;
	p2.y=point_min.y;
	p2.z=0;
	p3.x=point_max.x;
	p3.y=point_max.y;
	p3.z=0;
	p4.x=point_max.x;
	p4.y=point_min.y;
	p4.z=0;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_lowest;
	kdtree_lowest.setInputCloud(cloud_lowest);
	kdtree_lowest.nearestKSearch(p1, 1, point_indices, point_distances);
	p1.z=cloud_lowest->points[point_indices[0]].z;
	kdtree_lowest.nearestKSearch(p2, 1, point_indices, point_distances);
	p2.z=cloud_lowest->points[point_indices[0]].z;
	kdtree_lowest.nearestKSearch(p3, 1, point_indices, point_distances);
	p3.z=cloud_lowest->points[point_indices[0]].z;
	kdtree_lowest.nearestKSearch(p4, 1, point_indices, point_distances);
	p4.z=cloud_lowest->points[point_indices[0]].z;

	cloud_lowest->push_back(p1);
	cloud_lowest->push_back(p2);
	cloud_lowest->push_back(p3);
	cloud_lowest->push_back(p4);
//	pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\cloud_lowest.pcd", *cloud_lowest);

	//获取全部高程
	std::vector<float> height;
	TINConstruct TC;
	TC.construct_grid(cloud_lowest, input_cloud, temp_cloud, height);
	for(int i=0;i<height.size();i++)
	{
		if(height[i]>8.0)
		{
			seed_cloud_temp->push_back(temp_cloud->points[i]);
		}
	}
}